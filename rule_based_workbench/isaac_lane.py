"""Task-owned Isaac observation/replay adapter; run with the pinned Isaac Python."""
from __future__ import annotations

import argparse
import hashlib
import inspect
import math
from dataclasses import asdict, replace
from copy import deepcopy
import json
from real_contract import is_device_source
import os
import shutil
import sys
from pathlib import Path
import time
import traceback
from types import MethodType, SimpleNamespace

from contract import demo_state
from integration import CUROBO_PLANNER_ID, DualGpuPipeline, ObservationWindow, PIPELINE_LANES, _digest
from live_preview import PREVIEW_CAMERAS, PreviewPublisher, camera_frame_path, publish_rgb


# The forced path is deliberately narrow: it records contact failures while
# keeping the simulator running. Action validation, timeouts, and backend
# failures still terminate normally.
DIAGNOSTIC_COLLISION_REASONS = frozenset({
    "folded_self_collision", "robot_forbidden_contact", "cup_machine_forbidden_contact",
    "finger_cup_forbidden_contact", "cup_table_recontact_or_invalid_support",
    "cup_unknown_contact", "contact_force_limit", "release_force_limit",
})


def enable_forced_diagnostic_replay(runtime):
    """Keep collision diagnostics without changing PhysX or action guards."""
    if not hasattr(runtime, "evaluator"):
        raise ValueError("forced diagnostics require the Isaac interaction evaluator")
    evaluator = runtime.evaluator
    events = []
    original_fail = evaluator.fail
    original_observe = evaluator.observe

    def fail(reason, *, status="completed"):
        event = original_fail(reason, status=status)
        if reason in DIAGNOSTIC_COLLISION_REASONS:
            events.append({"reason": reason, "status": status,
                           "physics_step": getattr(evaluator, "_current_physics_step", None)})
            # The native evaluator has already recorded violations/audit. Clear
            # only its terminal latch so the next physics tick is observable.
            evaluator.event = None
            return None
        return event

    def observe(*args, **kwargs):
        event = original_observe(*args, **kwargs)
        if event is not None and event.reason == "success":
            events.append({"reason": "success", "status": event.run_status,
                           "physics_step": getattr(evaluator, "_current_physics_step", None)})
            evaluator.event = None
            return None
        return event

    evaluator.fail = fail
    evaluator.observe = observe
    runtime._forced_diagnostic_events = events
    return events


def write(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".partial")
    with temp.open("x") as f:
        json.dump(value, f, allow_nan=False)
    if path.exists():
        raise FileExistsError(path)
    temp.rename(path)


def append_jsonl(path, value):
    """Append one complete telemetry row; a partial final line is never emitted."""
    with Path(path).open("a") as stream:
        stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()


def _number(value):
    try:
        if hasattr(value, "item"):
            value = value.item()
        return float(value)
    except (AttributeError, TypeError, ValueError):
        return None


def _norm(value):
    try:
        values = value.detach().cpu().numpy().reshape(-1).tolist() if hasattr(value, "detach") else list(value)
        values = [_number(item) for item in values]
        return math.sqrt(sum(item * item for item in values)) if values and all(item is not None for item in values) else None
    except (AttributeError, TypeError, ValueError):
        return None


def replay_telemetry(runtime, stage, tick, target, report, measured_state, *, distance=False):
    """Keep available native signals and explicit nulls when a runtime has no sensor field."""
    sample, _ = runtime.read(runtime.loop.physics_steps)
    audit = getattr(getattr(runtime, "evaluator", None), "last_audit", {}) or {}
    facts = getattr(getattr(runtime, "evaluator", None), "facts", {}) or {}
    event = report.get("event") or {}
    target_open = max(target["left_gripper"], target["right_gripper"])
    reason = event.get("reason")
    if reason == "grasp_lost":
        grasp_state = "lost"
    elif distance:
        from gripper_distance_plan import HOLDING_STAGES
        grasp_state = 'closing' if stage=='close_ramp' else 'holding' if stage in HOLDING_STAGES else 'release_transition'
    elif stage in {"open", "separate", "release_hold"} or target_open > 0:
        grasp_state = "release_transition"
    else:
        grasp_state = "holding"
    return {
        "tick": tick["tick"], "time_s": tick["time_s"], "stage": stage,
        "target_gripper_opening_mm": {"left": target["left_gripper"], "right": target["right_gripper"]},
        "jaw_opening_mm": {"left": measured_state["left_gripper"], "right": measured_state["right_gripper"]},
        "cup_velocity_m_s": _norm(getattr(sample, "cup_velocity", None)),
        "finger_contact_count": audit.get("active_finger_contact_count"),
        "contact_count": audit.get("contact_count"),
        "support_force_n": audit.get("support_force_n"),
        "finger_force_n": (sum((audit.get("finger_forces_n") or {}).values())
                           if isinstance(audit.get("finger_forces_n"), dict) else None),
        "oracle_facts": {key: bool(value) for key, value in facts.items()},
        "grasp_state": grasp_state, "event": reason, "actual_substeps": report.get("actual_substeps"),
        "grasp_state_basis": 'native_grasp_lost_event' if reason == 'grasp_lost' else 'command_stage' if distance else 'legacy_binary_target',
        "requested_substeps": report.get("requested_substeps"), "terminated": bool(report.get("terminated")),
        "action_applied": bool(report.get("action_applied")), "measured": measured_state,
    }


def wait_for(path, seconds=600):
    deadline = time.monotonic() + seconds
    while True:
        for side in ("simulated-real", "dsw-scene"):
            failed = path.parent / (side + "-failed.json")
            if failed.exists():
                raise RuntimeError(f"peer execution failed: {failed}: {json.loads(failed.read_text()).get('error', 'unknown error')}")
        if path.exists():
            return json.loads(path.read_text())
        if time.monotonic() > deadline:
            raise TimeoutError(str(path))
        time.sleep(0.01)


def confirm_tick(barrier, side, tick):
    """One completion barrier bounds peer skew to one tick; failures never release it."""
    other = "dsw-scene" if side == "simulated-real" else "simulated-real"
    write(barrier / f"{side}-{tick}-done.json", {"tick": tick})
    result = wait_for(barrier / f"{other}-{tick}-done.json", 60)
    if result != {"tick": tick}:
        raise ValueError("peer completion tick mismatch")


def configure_observation_scene(cfg):
    """All three exported observation NPZ files require RGB and optical depth."""
    for name in ("d405_base_0", "d405_left_wrist_0", "d405_right_wrist_0"):
        camera = getattr(cfg.scene, name, None)
        if camera is None:
            raise ValueError(f"observation requires camera {name}")
        camera.data_types = ["rgb", "distance_to_image_plane"]


def validate_observation_outputs(scene):
    for name in ("d405_base_0", "d405_left_wrist_0", "d405_right_wrist_0"):
        missing = {"rgb", "distance_to_image_plane"} - set(scene[name].data.output)
        if missing:
            raise ValueError(f"observation camera {name} missing outputs: {', '.join(sorted(missing))}")


def preview_sensors(scene, *, whitebox_fast=False):
    """Reuse existing RGB buffers; do not create cameras or advance/render the scene."""
    if whitebox_fast:
        return {"base_0": scene["workspace_camera"]}
    return {camera: scene["d405_" + camera] for camera in PREVIEW_CAMERAS}


def calibrated_camera_rgb(env,camera):
    """Use the existing dataset projection, never expose its overscan buffer."""
    from robot_sim.tasks.manager_based.coffee_make.d405_camera import color_observation
    return color_observation(env,camera)[0,...,:3]


def calibrated_camera_k(env,camera):
    k=env.cfg.d405_rig['device_calibrations'][camera]['intrinsics']
    return [[k['fx'],0.,k['cx']],[0.,k['fy'],k['cy']],[0.,0.,1.]]


def calibrated_camera_depth(env,camera):
    """Sample depth with the same calibrated rays as RGB; preserve edge values."""
    from torch.nn.functional import grid_sample
    raw=env.scene['d405_'+camera].data.output['distance_to_image_plane']
    if raw.ndim==4:raw=raw[...,0]
    grid=env._d405_grid_cache[camera].expand(raw.shape[0],-1,-1,-1)
    return grid_sample(raw.unsqueeze(1).float(),grid,align_corners=True,mode='nearest')[0,0]


def publish_previews(publishers, sensors, metadata, *, whitebox_fast=False,env=None):
    for camera, sensor in sensors.items():
        pixels=sensor.data.output['rgb'][0,...,:3] if whitebox_fast else calibrated_camera_rgb(env,camera)
        publish_rgb(publishers[camera], pixels,
                    {**metadata, "camera": camera,
                     "sensor": "workspace_camera" if whitebox_fast else "d405_" + camera,
                     "image_domain": "render_pinhole" if whitebox_fast else "calibrated_color"})


def preview_stats(publishers):
    cameras = {camera: publisher.stats() for camera, publisher in publishers.items()}
    return {**cameras["base_0"], "cameras": cameras, "available_cameras": list(cameras)}


def physical(command, opening_mm=88.0):
    result = demo_state()
    for side in ("left", "right"):
        arm = getattr(command, side)
        result[side + "_arm_joints"] = list(arm.joint_positions_rad)
        result[side + "_gripper"] = opening_mm if arm.gripper_open else 0.0
    return result


class SimDistanceActionSpec:
    """Single-episode distance adapter; native arm limits and transition guard remain."""
    spec_id = 'ac_one_normalized_arms_gripper_travel_mm_v1'

    def __init__(self, native, opening_mm, release_contact_mm=None):
        self.native, self.opening_mm = native, opening_mm
        if release_contact_mm is not None and (not math.isfinite(release_contact_mm) or not 0<release_contact_mm<=opening_mm):
            raise ValueError('bounded nominal release contact gap required')
        self.release_contact_mm=release_contact_mm
        self.holding = None;self.stage=None

    def set_stage(self, stage):
        from gripper_distance_plan import HOLDING_STAGES
        if not isinstance(stage, str) or not stage:
            raise ValueError('explicit frozen command stage required')
        self.stage=stage;self.holding = stage in HOLDING_STAGES or stage=='close_ramp'

    def __getattr__(self, name):
        return getattr(self.native, name)

    def _binary(self, raw):
        if (len(raw) != 14 or any(type(v) not in (int, float) or not math.isfinite(v) for v in raw)
                or any(not 0 <= raw[slot] <= self.opening_mm for slot in (6, 13))):
            raise ValueError('explicit finite gripper travel targets required')
        values = list(raw)
        # Native evaluator consumes commanded_gripper_open. A positive distance
        # is not an open command: the frozen phase owns close versus release.
        holding=self.holding if self.holding is not None else raw[6]<self.opening_mm-1e-9
        if self.stage=='open' and self.release_contact_mm is not None and raw[6]<self.release_contact_mm-1e-9:
            holding=True  # Opening targets still below contact unload the preload.
        values[6] = -1.0 if holding else 1.0
        values[13] = 1.0 if raw[13] >= self.opening_mm-1e-9 else -1.0
        return values

    def decode(self, raw):
        return self.native.decode(self._binary(raw))

    def decode_single_arm_transition(self, previous, proposed, *, active_arm, absolute_tolerance=1e-9):
        if active_arm not in ('left', 'right'):
            raise ValueError('unknown active arm')
        inactive = 13 if active_arm == 'left' else 6
        old, new = self._binary(previous), self._binary(proposed)
        if abs(previous[inactive]-proposed[inactive]) > absolute_tolerance:
            raise ValueError('distance command would move the inactive gripper')
        return self.native.decode_single_arm_transition(old, new, active_arm=active_arm,
                                                        absolute_tolerance=absolute_tolerance)


def native_target(raw, spec, opening_mm, *, distance=False):
    """Decode native model coordinates, preserving distance-valued jaws verbatim."""
    decoder = SimDistanceActionSpec(spec, opening_mm) if distance and not isinstance(spec, SimDistanceActionSpec) else spec
    result = physical(decoder.decode(raw), opening_mm)
    if distance:
        result.update(left_gripper=raw[6], right_gripper=raw[13])
    return result


def simulation_initial_command(raw, spec, opening_mm, physics_step):
    """Record the actual reset command independently of noisy PhysX feedback."""
    if (len(raw) != 14 or any(type(v) not in (int, float) or not math.isfinite(v) for v in raw)
            or not math.isfinite(opening_mm) or opening_mm <= 0
            or type(physics_step) is not int or physics_step < 0):
        raise ValueError('finite native reset command and physics step required')
    return dict(schema_version='simulation_initial_command.v1', evidence_source='runtime.loop.previous',
        native_action=list(raw), target=native_target(raw, spec, opening_mm),
        action_spec_id=spec.spec_id, action_spec_sha256=_digest(asdict(spec)),
        opening_mm=opening_mm, physics_step=physics_step)


def distance_replay_previous(runtime, spec, plan):
    """Convert the previous command's encoding, never its commanded positions."""
    previous = list(runtime.loop.previous)
    decoded = native_target(previous, spec, plan['gripper_opening_mm'])
    if plan.get('planner_id') == CUROBO_PLANNER_ID:
        actual = simulation_initial_command(previous, spec, plan['gripper_opening_mm'], runtime.loop.physics_steps)
        frozen = plan.get('simulation_initial_command')
        if not isinstance(frozen, dict) or frozen.get('evidence_source') != 'runtime.loop.previous':
            raise ValueError('MotionGen replay requires an observed simulation reset command')
        for key in ('schema_version', 'native_action', 'target', 'action_spec_id', 'action_spec_sha256', 'opening_mm'):
            if frozen.get(key) != actual[key]:
                raise ValueError('source/target nominal reset commands differ: '+key)
    previous[6], previous[13] = decoded['left_gripper'], decoded['right_gripper']
    return previous


def install_distance_replay(env, runtime, spec, plan):
    """Keep guarded stepping/contact/raw paths, changing only the two jaw targets."""
    release_contact=None
    if plan.get('cup_contact_preset'):
        closing=plan['planning_validation']['gpu_final_acceptance']['closing']
        if closing.get('passed') is not True:raise ValueError('release requires accepted nominal contact geometry')
        release_contact=closing['end_m']*2000
    adapter = SimDistanceActionSpec(spec, plan['gripper_opening_mm'],release_contact_mm=release_contact)
    term = env.action_manager.get_term('guarded')
    robot = env.scene['robot']
    names = [*spec.left.arm.joint_names, 'left_joint7', 'left_joint8',
             *spec.right.arm.joint_names, 'right_joint17', 'right_joint18']
    if [robot.joint_names[i] for i in term.ids] != names:
        raise ValueError('distance replay action term joint order differs')
    limits = robot.data.joint_pos_limits[0, term.ids].tolist()
    if len(limits) != len(names):
        raise ValueError('distance replay joint limit dimensions differ')

    def targets(raw):
        command = adapter.decode(raw)
        return [*command.left.joint_positions_rad, raw[6]/2000, raw[6]/2000,
                *command.right.joint_positions_rad, raw[13]/2000, raw[13]/2000]

    for raw in plan['native_actions']:
        if any(not lo <= q <= hi for q, (lo, hi) in zip(targets(raw), limits)):
            raise ValueError('distance replay target exceeds measured native joint limits')

    def process_actions(current, actions):
        import torch
        if actions.shape != (1, 14) or not torch.isfinite(actions).all():
            raise ValueError('one finite 14D distance action required')
        raw = actions[0].tolist()
        adapter.decode_single_arm_transition(current._raw[0].tolist(), raw, active_arm='left')
        current._targets[:] = torch.tensor([targets(raw)], device=current.device)
        current._raw[:] = actions

    import torch
    previous = distance_replay_previous(runtime, spec, plan)
    adapter.decode(previous)
    runtime.loop.previous = tuple(previous)
    runtime.loop.action_spec = runtime.action = adapter
    term.spec = adapter
    term._raw[:] = torch.tensor([previous], dtype=term._raw.dtype, device=term.device)
    term.process_actions = MethodType(process_actions, term)
    return adapter


def incidental_claw_contacts(contacts, sample, acceptance, root, dt, *, grasp_confirmed, opening, diagnostic_compression=False):
    """Accept shallow metal contact as collision only; never credit it as a grasp."""
    if not math.isfinite(dt) or dt<=0:return tuple(contacts),[]
    cup=root+'/Cup';fingers={root+'/Robot/left_link7',root+'/Robot/left_link8'}
    forces=dict.fromkeys(fingers,0.)
    for contact in contacts:
        if cup in contact.actors:
            body=contact.actors[1-contact.actors.index(cup)]
            if body in forces:forces[body]+=math.sqrt(sum(v*v for v in contact.impulse))/dt
    opened=all(abs(q-acceptance.open_position_m)<=acceptance.open_tolerance_m for q in sample.jaw_positions)
    force_limit=min(acceptance.finger_force_max_n,acceptance.residual_force_max_n) if opening or opened else acceptance.finger_force_max_n
    safe=max(forces.values())<=force_limit and sample.finger_cup_relative_speed_m_s<=acceptance.finger_contact_speed_limit(grasp_confirmed)
    filtered=[];allowed=[]
    for contact in contacts:
        if safe and cup in contact.actors:
            i=1-contact.actors.index(cup);body=contact.actors[i]
            if (body in fingers and contact.shapes[i].startswith(body+'/collisions/')
                    and math.isfinite(contact.separation_m) and
                    (contact.separation_m>=-.003 or (diagnostic_compression and
                     contact.shapes[1-i] in {cup+'/Collisions/MiddleBody',cup+'/Collisions/UpperBody'}))):
                allowed.append(contact);continue
        filtered.append(contact)
    return tuple(filtered),allowed


def install_grasp_contact_adapter(runtime,*,diagnostic_compression=False):
    """One shared body/metal policy for reset, local preset probes and replay."""
    if hasattr(runtime,'_grasp_contact_adapter'):
        return runtime._grasp_contact_adapter
    evaluator=runtime.evaluator
    height_epsilon=.0001 if diagnostic_compression else 0.
    receipt=dict(planning_recommendation_m=list(evaluator.cup.candidate_grasp_band_m),
                 accepted_body_band_m=[.035,.100],allowed_shapes=['MiddleBody','UpperBody'],
                 contact_height_numerical_tolerance_m=height_epsilon,
                 rim_or_lid_grasp_credit=False,diagnostic_compression_observation=diagnostic_compression,
                 incidental_contacts=dict(count=0,max_penetration_m=0.))
    original=evaluator.observe
    evaluator.cup=replace(evaluator.cup,candidate_grasp_band_m=(.035-height_epsilon,.100+height_epsilon))
    def observe(*args,**kwargs):
        sample=args[0] if args else kwargs['sample']
        if getattr(runtime,'root',None):
            contacts=args[2] if len(args)>2 else kwargs['contacts']
            cup_root=runtime.root+'/Cup'
            for contact in contacts:
                if cup_root not in contact.actors:continue
                ci=contact.actors.index(cup_root);ri=1-ci;body=contact.actors[ri]
                if (contact.shapes[ri]==body+'/RealFingerRubberPad'
                        and body in {runtime.root+'/Robot/left_link7',runtime.root+'/Robot/left_link8'}
                        and contact.shapes[ci] not in {cup_root+'/Collisions/MiddleBody',cup_root+'/Collisions/UpperBody'}
                        and math.sqrt(sum(v*v for v in contact.impulse))>0):
                    evaluator._current_physics_step=sample.physics_step
                    return evaluator.fail('finger_cup_forbidden_contact')
            filtered,allowed=incidental_claw_contacts(contacts,sample,evaluator.spec,runtime.root,kwargs['dt'],
                grasp_confirmed=evaluator.history['grasp_confirmed'],opening=kwargs.get('commanded_gripper_open',False),
                diagnostic_compression=diagnostic_compression)
            receipt['incidental_contacts']['count']+=len(allowed)
            receipt['incidental_contacts']['max_penetration_m']=max(receipt['incidental_contacts']['max_penetration_m'],
                max((max(0.,-c.separation_m) for c in allowed),default=0.))
            if len(args)>2:args=(*args[:2],filtered,*args[3:])
            else:kwargs={**kwargs,'contacts':filtered}
        return original(*args,**kwargs)
    evaluator.observe=observe;runtime._grasp_contact_adapter=receipt
    return receipt


def tray_placement_region(evaluator, sample, scene):
    """Full cup footprint over the tray; shared by contact and planned release."""
    a=evaluator.spec;inverse=scene.world_from_support.inverse()
    pose=inverse.compose(sample.world_from_cup);bottom=pose.position
    polygon=[inverse.transform_point(p) for p in scene.support_polygon_world_m]
    if len(polygon)<3:return False
    radius=evaluator.cup.bottom_radius_m+a.edge_margin_m
    return (pose.rotate((0,0,1))[2]>=math.cos(a.tilt_rad)
            and evaluator._lowest_z(pose)>=-a.surface_tolerance_m
            and all((q[0]-p[0])*(bottom[1]-p[1])-(q[1]-p[1])*(bottom[0]-p[0])
                >=radius*math.hypot(q[0]-p[0],q[1]-p[1])
                for p,q in zip(polygon,polygon[1:]+polygon[:1])))


def legal_tray_contact(evaluator, sample, scene, contacts, dt):
    """Current bottom contact and full tray footprint; no mesh/force-threshold fit."""
    if not math.isfinite(dt) or dt<=0 or not tray_placement_region(evaluator,sample,scene):return False
    a=evaluator.spec;inverse=scene.world_from_support.inverse()
    pose=inverse.compose(sample.world_from_cup)
    if abs(evaluator._lowest_z(pose))>a.surface_tolerance_m:return False
    cup_inverse=sample.world_from_cup.inverse();force=0.
    for contact in contacts:
        if evaluator.cup_root not in contact.actors:continue
        i=contact.actors.index(evaluator.cup_root)
        if contact.shapes[1-i]!=scene.support_shape_path:continue
        value=math.sqrt(sum(v*v for v in contact.impulse))/dt
        if value<=0:continue
        local=cup_inverse.transform_point(contact.position);normal=cup_inverse.rotate(contact.normal)
        p=inverse.transform_point(contact.position);n=inverse.rotate(contact.normal)
        if (abs(local[2])>a.surface_tolerance_m or abs(normal[2])<math.cos(a.tilt_rad)
                or abs(p[2])>a.surface_tolerance_m or abs(n[2])<math.cos(a.tilt_rad)):return False
        force+=value
    return math.isfinite(force) and 0<force<=a.support_force_max_n


def install_motiongen_replay_policy(runtime, plan, *, applied_context=None):
    """Frozen success timing and explicit, contact-scoped supported unloading."""
    from gripper_distance_plan import NATIVE_DISTANCE_ENCODING
    if applied_context is not None and not callable(applied_context):
        raise ValueError('applied command context must be callable')
    if (plan.get('planner_id') != CUROBO_PLANNER_ID or plan.get('schema_version') != 'dual_gpu_pipeline.v1'
            or plan.get('native_action_encoding') != NATIVE_DISTANCE_ENCODING):
        raise ValueError('success scheduling is limited to MotionGen simulation distance replay')
    evaluator = runtime.evaluator
    if (hasattr(runtime, '_motiongen_replay_policy') or hasattr(runtime, '_forced_diagnostic_events')
            or getattr(evaluator, 'diagnostic_relax_guards', False) or getattr(evaluator, 'diagnostic_observe_only', False)
            or evaluator.event is not None or runtime.loop.terminal):
        raise ValueError('fresh non-diagnostic runtime required for frozen replay policy')
    base = evaluator.spec
    declared = plan.get('native_identity', {}).get('acceptance')
    if not isinstance(declared, dict) or _digest(asdict(base)) != _digest(declared):
        raise ValueError('runtime base acceptance differs from frozen plan')
    hz, nominal = plan['control']['physics_hz'], plan['control']['substeps']
    counts, ticks, stages = plan['native_substeps'], plan['ticks'], plan['stages']
    if (type(hz) is not int or type(nominal) is not int or nominal <= 0 or hz != 30*nominal
            or len(counts) != len(ticks) or not counts
            or any(type(n) is not int or not 0 < n <= nominal for n in counts)
            or any(n != nominal for n in counts[:-1])):
        raise ValueError('frozen replay requires complete native physics cadence')
    starts = [stage['start_tick'] for stage in stages]
    if (not starts or starts[0] != 0 or any(type(n) is not int or not 0 <= n < len(ticks) for n in starts)
            or any(b <= a for a, b in zip(starts, starts[1:]))):
        raise ValueError('frozen stages must cover the command timeline')
    opening = [i for i, stage in enumerate(stages) if stage['name'] == 'open']
    if len(opening) != 1:
        raise ValueError('one explicit frozen opening stage required')
    index = opening[0]
    begin, end = starts[index], starts[index+1] if index+1 < len(starts) else len(ticks)
    rows = [tick['target'] for tick in ticks[begin:end]]
    policy = plan['gripper_distance_control']
    closed = policy['closed_metal_gap_mm']-policy['native_travel_offset_mm']
    if (abs(rows[0]['left_gripper']-closed) > 1e-9
            or abs(rows[-1]['left_gripper']-plan['gripper_opening_mm']) > 1e-9
            or any(any(row[key] != rows[0][key] for key in ('left_arm_joints', 'right_arm_joints', 'right_gripper')) for row in rows)
            or any(not 0 <= b['left_gripper']-a['left_gripper'] <= .20000001 for a, b in zip(rows, rows[1:]))):
        raise ValueError('release budget requires the declared stationary continuous open ramp')
    start = runtime.loop.physics_steps
    if type(start) is not int or start < 0 or evaluator.previous.physics_step != start:
        raise ValueError('evaluator and execution loop must share the starting physics step')
    open_steps = sum(counts[begin:end])
    deadline = start+sum(counts)
    effective = replace(base, release_timeout_s=base.release_timeout_s+open_steps/hz)
    configuration = dict(schema_version='motiongen_frozen_replay_policy.v1', timeline_digest=plan['timeline_digest'],
        start_physics_step=start, final_physics_step=deadline,
        success_policy='defer success until final substep; final native observe must currently succeed',
        failure_policy='all native failures, force/contact limits, instability and actual timeout remain immediate',
        base_acceptance=asdict(base), effective_acceptance=asdict(effective),
        base_acceptance_sha256=_digest(asdict(base)), effective_acceptance_sha256=_digest(asdict(effective)),
        base_acceptance_native_digest=base.digest(), effective_acceptance_native_digest=effective.digest(),
        release_timeout_base_s=base.release_timeout_s, planned_open_duration_s=open_steps/hz,
        release_timeout_effective_s=effective.release_timeout_s,
        open_start_tick=begin, open_end_tick_exclusive=end,
        first_open_physics_step=start+sum(counts[:begin])+1, last_open_physics_step=start+sum(counts[:end]),
        physical_configuration_sha256=runtime.physical_sha256,
        physical_configuration_unchanged=True,
        final_native_success_required=True,
        final_gripper_travel_mm=ticks[-1]['target']['left_gripper'],
        restored_nonopen_jaw_may_prevent_success=abs(ticks[-1]['target']['left_gripper']/2000-base.open_position_m) > base.open_tolerance_m)
    if applied_context is not None:
        configuration.update(execution_source='owner_applied_command', final_physics_step=None,
            success_policy='current native success at the actual Owner completion feedback step',
            first_open_physics_step=None,last_open_physics_step=None,
            initialization_scope='source_tick=None; no parent-plan tick or geometry qualification')
    if plan.get('cup_contact_preset'):
        configuration['release_preload_unload_until_model_gap_mm']=plan['planning_validation']['gpu_final_acceptance']['closing']['end_m']*2000
        configuration['release_preload_unload_scope']='simulation diagnosis; unchanged targets, grip force cap and2N release cap'
    configuration['incidental_claw_cup_collision']=dict(max_penetration_m=.003,shape='left finger metal meshes',grasp_credit=False,
        force_and_speed_limits='existing native contact and residual-force limits')
    grasp_policy=install_grasp_contact_adapter(runtime)
    configuration['rubber_pad_body_contact']={k:v for k,v in grasp_policy.items() if k!='incidental_contacts'}
    receipt = dict(configuration=configuration, configuration_sha256=_digest(configuration),
                   progress=dict(deferred_success_count=0, first_deferred_success_step=None,
                                 last_observed_step=start, final_native_success=False))
    receipt['incidental_contacts']=grasp_policy['incidental_contacts']
    receiving_goal=plan.get('task_targets',{}).get('placement_goal')
    if receiving_goal is not None:
        from robot_sim.tasks.manager_based.coffee_make.outlet_alignment import OUTLET_TOLERANCE_M
        if (receiving_goal.get('schema_version')!='cup_outlet_goal.v1'
                or receiving_goal.get('metric')!='cup_mouth_world_xy_to_liquid_outlet'
                or receiving_goal.get('tolerance_m')!=OUTLET_TOLERANCE_M):
            raise ValueError('unsupported frozen receiving goal')
        configuration['placement_goal']=deepcopy(receiving_goal)
        receipt['configuration_sha256']=_digest(configuration)
    original_observe = evaluator.observe
    original_fail=evaluator.fail
    placement_transition=[False]
    observed_transition_failure=[False]
    applied_execution_id=[None]
    receipt['progress'].update(placement_transition_substeps=0,
        transition_grasp_loss_observations=0,transition_slip_observations=0,
        max_transition_relative_speed_m_s=0.,max_transition_cup_speed_m_s=0.)
    diagnostic_cup=bool(plan.get('cup_contact_preset'))
    if diagnostic_cup:
        configuration['failure_policy']='grip loss/slip and transient speed are observations during scoped placement only; native collision/force/final stability remain required'
        configuration['supported_contact_policy']=dict(
            landing_contact_speed='observed during legal tray contact; no instantaneous speed rejection',
            airborne_relative_speed_unchanged=True,
            tray_release_neighborhood_m=base.slip_m,
            tray_release_neighborhood_source='legacy slip_m value; distinct from airborne relative-slip semantics',
            unload='legal loaded tray contact while lowering, or planned stationary open/release_hold over full tray footprint; no prior force confirmation',
            final_support_force_and_stability_unchanged=True,
            authorization='user20261008 approved held lowering, passive tray yielding and active-release separation without transient-speed/grip constraints')
        receipt['configuration_sha256']=_digest(configuration)

    def fail(reason, **kwargs):
        if reason in {'grasp_lost','grasp_slip'} and placement_transition[0]:
            observed_transition_failure[0]=True
            key='transition_grasp_loss_observations' if reason=='grasp_lost' else 'transition_slip_observations'
            receipt['progress'][key]+=1
            return None
        return original_fail(reason,**kwargs)
    if diagnostic_cup:evaluator.fail=fail

    def observe(*args, **kwargs):
        sample = args[0] if args else kwargs['sample']
        step = sample.physics_step
        if applied_context is None and step > deadline:
            raise RuntimeError('native observation exceeded frozen replay deadline')
        scene=args[1] if len(args)>1 else kwargs.get('scene')
        contacts=args[2] if len(args)>2 else kwargs.get('contacts',())
        if applied_context is None:
            tick_index=min(len(ticks)-1,(step-start-1)//nominal)
            stage=next(row['name'] for row in reversed(stages) if row['start_tick']<=tick_index)
            final_sample=step==deadline
        else:
            context=applied_context(sample)
            if not isinstance(context,dict) or type(context.get('final_sample')) is not bool:
                raise ValueError('actual applied source tick/stage/final-sample context required')
            identity=context.get('execution_id')
            if (not isinstance(identity,str) or not identity or
                    applied_execution_id[0] not in (None,identity)):
                raise ValueError('applied execution identity changed')
            applied_execution_id[0]=identity
            tick_index=context.get('source_tick');stage=context.get('stage')
            if tick_index is None:
                if stage!='initialize':raise ValueError('no parent tick is only valid for initialization')
            elif (type(tick_index) is not int or not 0<=tick_index<len(ticks) or
                    stage!=next(row['name'] for row in reversed(stages) if row['start_tick']<=tick_index)):
                raise ValueError('applied stage differs from the frozen parent source tick')
            final_sample=context['final_sample']
            if final_sample and tick_index!=len(ticks)-1:
                raise ValueError('final task sample requires the final actually applied parent command')
            receipt['progress'].update(last_observed_source_tick=tick_index,
                last_observed_execution_id=context.get('execution_id'))
        contact_ok=(diagnostic_cup and stage in {'lower','support_settle','open','release_hold'}
            and scene is not None and legal_tray_contact(evaluator,sample,scene,contacts,kwargs['dt']))
        releasing=(diagnostic_cup and stage in {'open','release_hold'}
            and evaluator.history.get('grasp_confirmed',False)
            and evaluator.history.get('table_separated',False)
            and scene is not None and tray_placement_region(evaluator,sample,scene)
            # Legacy slip_m also supplies the near-tray height allowance. A
            # change to airborne slip therefore requires reviewing both uses;
            # the planned bottom goal stays on the tray, with no added gap.
            and evaluator._lowest_z(scene.world_from_support.inverse().compose(sample.world_from_cup))<=base.slip_m)
        placement_transition[0]=(contact_ok or releasing) and not evaluator.history.get('release_confirmed',False)
        if placement_transition[0]:
            speed=math.sqrt(sum(v*v for v in sample.cup_velocity))
            relative=sample.finger_cup_relative_speed_m_s
            receipt['progress']['placement_transition_substeps']+=1
            for key,value in [('max_transition_relative_speed_m_s',relative),('max_transition_cup_speed_m_s',speed)]:
                receipt['progress'][key]=max(receipt['progress'][key],value)
            # Native finite-spec fields are comparison bounds. Bind them to
            # this actual sample instead of inventing a higher speed threshold;
            # preserve force, geometry and final stability fields verbatim.
            evaluator.spec=replace(effective,landing_speed_m_s=max(base.pregrasp_contact_speed_m_s,speed),
                contact_speed_m_s=max(base.contact_speed_m_s,relative),
                pregrasp_contact_speed_m_s=max(base.pregrasp_contact_speed_m_s,relative))
        else:evaluator.spec=effective
        observed_transition_failure[0]=False
        try:
            event = original_observe(*args, **kwargs)
            if event is None and observed_transition_failure[0]:
                # Native loss/slip returns before advancing previous. Keep the
                # actual sample current after this scoped, observed transition.
                evaluator.previous=sample
        finally:
            evaluator.spec=effective;placement_transition[0]=False
        receipt['progress']['last_observed_step'] = step
        if applied_context is not None:
            receipt['progress']['last_sample_native_success']=False
        if event is not None and event.reason == 'success':
            if receiving_goal is not None:
                from robot_sim.tasks.manager_based.coffee_make.outlet_alignment import outlet_distance
                try:distance=outlet_distance(scene,evaluator.cup,sample.world_from_cup)
                except ValueError:return original_fail('outlet_geometry_invalid',status='unverifiable')
                passed=distance<=receiving_goal['tolerance_m']
                receipt['progress']['receiving_goal']=dict(passed=passed,distance_m=distance,
                    tolerance_m=receiving_goal['tolerance_m'],physics_step=step)
                if not passed:return original_fail('outlet_misaligned')
            if evaluator.event is not event:
                raise RuntimeError('native success event and terminal latch differ')
            if applied_context is not None:
                receipt['progress']['last_sample_native_success']=True
            if not final_sample:
                receipt['progress']['deferred_success_count'] += 1
                if receipt['progress']['first_deferred_success_step'] is None:
                    receipt['progress']['first_deferred_success_step'] = step
                evaluator.event = None  # Only this newly emitted success; failures never reach this branch.
                return None
            receipt['progress']['final_native_success'] = True
        return event

    evaluator.spec = effective  # Keep frozen runtime.spec.acceptance and physical identity unchanged.
    evaluator.observe = observe
    runtime._motiongen_replay_policy = receipt
    if applied_context is not None:
        runtime._motiongen_applied_context=applied_context
        runtime._motiongen_final_source_tick=len(ticks)-1
    return receipt


def finish_motiongen_replay_policy(runtime):
    receipt = runtime._motiongen_replay_policy
    event = runtime.evaluator.event
    applied_context=getattr(runtime,'_motiongen_applied_context',None)
    if applied_context is not None:
        context=applied_context(runtime.evaluator.previous)
        progress=receipt['progress']
        if (context.get('final_sample') is not True or
                context.get('source_tick')!=runtime._motiongen_final_source_tick or
                context.get('execution_id')!=progress.get('last_observed_execution_id') or
                progress.get('last_observed_source_tick')!=context['source_tick'] or
                progress['last_observed_step']!=runtime.loop.physics_steps or
                runtime.evaluator.previous.physics_step!=runtime.loop.physics_steps or
                progress.get('last_sample_native_success') is not True or
                getattr(runtime.evaluator,'first_failure_reason',None) is not None or
                event is not None and event.reason!='success'):
            raise ValueError('Owner completed without current final-step native task success')
        progress['final_native_success']=True
        receipt['configuration']['actual_final_physics_step']=runtime.loop.physics_steps
        receipt['configuration_sha256']=_digest(receipt['configuration'])
        return receipt
    if (runtime.loop.physics_steps != receipt['configuration']['final_physics_step']
            or receipt['progress']['last_observed_step'] != runtime.loop.physics_steps
            or not receipt['progress']['final_native_success']
            or event is None or event.reason != 'success'):
        raise RuntimeError('frozen timeline ended without current native success at its final physics substep')
    return receipt


def validate_geometry_inputs(args, source, suite):
    if (args.side != 'dsw-scene' or source.get('schema_version') != 'dual_gpu_pipeline.v1'
            or source.get('adapter') != 'foundationpose' or getattr(args, 'hardware_binding', None)
            or getattr(args, 'independent_replay', False) or getattr(args, 'force_diagnostic_replay', False)):
        raise ValueError('MotionGen scene export requires the detected dual-DSW source only')
    bridge = DualGpuPipeline(); bridge.ingest_source(source)
    episodes = suite.get('episodes', [])
    keys = ('episode_id', 'seed', 'machine_asset', 'environment_from_machine_root', 'environment_from_cup')
    if (len(episodes) != 1 or {k: episodes[0].get(k) for k in keys} != source['scene']['episode_scene']
            or episodes[0].get('randomization') is not None):
        raise ValueError('MotionGen scene must reset the frozen detected scene without randomization')
    if (not args.finger_envelope or not args.finger_envelope_sha256
            or file_sha256(args.finger_envelope) != args.finger_envelope_sha256):
        raise ValueError('scene export requires the pinned native finger envelope')


def export_geometry_inputs(args, source, runtime, episode, control, acceptance, info, measured):
    """Export reset geometry only; no old planner.initial/inverse/path search call."""
    import isaaclab.sim as sim_utils
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
    from robot_sim.policies.rule_coffee import RuleObservation
    from robot_sim.tasks.manager_based.coffee_make.episode_planner import EpisodePlanner
    from robot_sim.tasks.manager_based.coffee_make.layout import canonical_coffee_make_v1_layout
    from robot_sim.tasks.manager_based.coffee_make.native_finger_geometry import export_native_fingers
    from robot_sim.tasks.manager_based.coffee_make.scene_instance import ResolvedPrimitive
    native = export_native_fingers(sim_utils.get_current_stage(), runtime.root+'/Robot')
    if not native['complete']:
        raise ValueError('incomplete native geometry')
    digest = hashlib.sha256(json.dumps(dict(parts=native['parts'], planes=native['native_planes']), sort_keys=True).encode()).hexdigest()
    if getattr(args,'rebuild_finger_envelope',False):
        # Model preparation only: use the existing native vertex/plane proof,
        # preserving the old input cache and retaining the constructor's guard.
        import runpy
        builder=Path(inspect.getfile(EpisodePlanner)).resolve().parents[6]/'scripts/build_coffee_m1_finger_envelope.py'
        envelope=runpy.run_path(str(builder))['envelope']
        prepared=deepcopy(native);checks=[]
        for part in prepared['parts']:
            vertices,faces,audit=envelope(part['vertices'],prepared['native_planes'][part['name']])
            part['vertices'],part['faces']=vertices,faces;checks.append(dict(name=part['name'],**audit))
        prepared.update(native_geometry_sha256=digest,representation='native_vertex_and_plane_envelope',containment=checks,
                        builder_sha256=file_sha256(builder),source_cache_sha256=args.finger_envelope_sha256,
                        robot_usd_sha256=file_sha256(ac_one_asset_spec(args.assets).usd_path))
        args.finger_envelope=args.output/'finger-envelope.json';write(args.finger_envelope,prepared)
        args.finger_envelope_sha256=file_sha256(args.finger_envelope)
    layout = canonical_coffee_make_v1_layout()
    table = ResolvedPrimitive(runtime.evaluator.table_shape, runtime.evaluator.table_shape, 'shell',
                             RigidTransform(layout.worktable.position), layout.worktable_size_m, 'box', None)
    parameters = inspect.signature(EpisodePlanner).parameters
    options = {name: value for name, value in dict(episode=episode, open_position_m=acceptance.open_position_m).items()
               if name in parameters}
    # Constructor supplies the pinned geometry/support identities; it does not search.
    planner = EpisodePlanner(args.assets, args.finger_envelope, args.finger_envelope_sha256, digest, table, **options)
    sample, scene = runtime.read(runtime.loop.physics_steps)
    observation = RuleObservation(info['episode_id'], info['step_id'], dict(info['oracle_facts']), sample, scene,
        ((info.get('execution_report') or {}).get('event') or {}).get('reason'), bool(info['contact_state']['support_acquired']))
    identity = dict(control=asdict(control), acceptance=asdict(acceptance), physical_configuration=runtime.physical_identity,
        physical_configuration_sha256=runtime.physical_sha256, finger_envelope_sha256=args.finger_envelope_sha256,
        native_geometry_sha256=digest, planner_identity=planner.identity, geometry_export_only=True, search_executed=False)
    write(args.output/'planning-input.json', dict(parts=[asdict(p) for p in planner.parts], cup=asdict(planner.cup),
        table=asdict(table), obstacles=[asdict(p) for p in planner.obstacles], observation=asdict(observation),
        planner_identity=planner.identity, open_position_m=acceptance.open_position_m,
        source_digest=_digest(source), native_identity=identity,
        urdf_sha256=file_sha256(ac_one_asset_spec(args.assets).urdf_path),
        simulation_initial_state=measured,
        simulation_initial_command=simulation_initial_command(runtime.loop.previous, runtime.loop.action_spec,
            acceptance.open_position_m*2000, runtime.loop.physics_steps),
        geometry_export_only=True, search_executed=False,
        world_from_robot=asdict(scene.instance.world_from_environment.compose(scene.instance.environment_from_robot))))
    write(args.output/'result.json', dict(status='geometry-inputs-ready', source_digest=_digest(source),
        geometry_export_only=True, search_executed=False, evidence_source='simulation'))


def file_sha256(path):
    with Path(path).open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def raw_digest(value):
    # The native raw schema hashes UTF-8 JSON without ASCII escaping.
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def validate_reset_grippers(measured, episode, opening_mm, *, tolerance_mm=.5):
    state=(getattr(episode,'metadata',None) or {}).get('initial_robot_state')
    expected={side:state[side+'_gripper'] if state else opening_mm for side in ('left','right')}
    for side,gap in expected.items():
        actual=measured[side+'_gripper']
        if not all(math.isfinite(v) for v in (gap,actual)) or not 0<=gap<=opening_mm or abs(actual-gap)>tolerance_mm:
            raise ValueError('actual reset gripper differs from the declared initial state: '+side)
    return expected


def runtime_profile_snapshot(capture, completed_ticks):
    profile=getattr(capture,'profile',None)
    if profile is None:return None
    return dict(clock='perf_counter_host_wall_no_added_cuda_sync',completed_ticks=completed_ticks,
                rows=deepcopy(profile.rows),scope='running capture; nested inclusive times overlap')


def bind_effective_gripper_identity(runtime, open_position_m, gripper_limits):
    # Match the native runner: retain the authored base and bind this episode's reset override.
    runtime.physical_identity["effective_gripper_opening_mm"] = open_position_m * 2000.0
    runtime.physical_identity["effective_gripper_open_position_m"] = open_position_m
    runtime.physical_identity["effective_gripper_joint_limits_m"] = gripper_limits
    runtime.physical_identity["gripper_limit_source"] = "PhysX readback; development displacement range"
    runtime.physical_sha256 = hashlib.sha256(
        json.dumps(runtime.physical_identity, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def validate_physical_identity(runtime, plan):
    identity = plan.get("native_identity", {})
    if (identity.get("physical_configuration_sha256") != runtime.physical_sha256
            or _digest(identity.get("physical_configuration")) != _digest(runtime.physical_identity)):
        raise ValueError("replay physical configuration differs from the frozen native planner")


def replay_handshake(args):
    if getattr(args, "independent_replay", False):
        return
    if not args.barrier:
        raise ValueError("dual replay requires peer barrier directory")
    write(args.barrier / (args.side + "-ready.json"), {"run_id": args.run_id, "time_ns": time.time_ns()})
    other = "dsw-scene" if args.side == "simulated-real" else "simulated-real"
    wait_for(args.barrier / (other + "-ready.json"))


def replay_tick_sync(args, tick):
    if not getattr(args, "independent_replay", False):
        confirm_tick(args.barrier, args.side, tick)


def replay_provenance(plan):
    return {"source_run_id": plan["source_run_id"], "plan_run_id": plan["run_id"],
            "timeline_digest": plan["timeline_digest"], "evidence_source": "simulation"}


def replay_scheduling(args):
    return ("independent simulation replay" if getattr(args, "independent_replay", False)
            else "CPFS per-tick lockstep, not real-time")


def validate_record_options(args):
    independent = getattr(args, "independent_replay", False)
    binding = getattr(args, "hardware_binding", None)
    if independent or binding:
        if (not independent or args.mode!='replay' or args.side not in {'simulated-real','dsw-scene'}
                or getattr(args,'force_diagnostic_replay',False) or getattr(args,'barrier',None)
                or (binding and args.side!='dsw-scene')):
            raise ValueError('independent replay requires a simulation lane, no peer barrier or forced diagnostics; physical binding is target-only')
    if getattr(args, "force_diagnostic_replay", False):
        if args.mode != "replay" or args.record_raw:
            raise ValueError("forced diagnostics require replay without raw capture")
        if args.side not in {"simulated-real", "dsw-scene"}:
            raise ValueError("forced diagnostics require an Isaac simulation lane")
    if not args.record_raw:
        return None
    if args.mode != "replay" or args.raw_prompt_file is None:
        raise ValueError("--record-raw requires replay mode and --raw-prompt-file")
    prompt = json.loads(args.raw_prompt_file.read_text())
    if not isinstance(prompt, dict) or any(not isinstance(prompt.get(k), str) or not prompt[k].strip()
                                           for k in ("text", "language", "version")):
        raise ValueError("raw prompt requires nonempty text, language and version")
    return prompt


def validate_replay_inputs(args, plan, source, suite):
    expected = PIPELINE_LANES.get(args.side)
    if getattr(args,'execution_json',None):
        from local_execution import validate
        resource=validate({'execution':json.loads(args.execution_json)})
        expected=dict(instance_id=resource['instance_id'],gpu=resource['gpu_index'])
    if not expected or (args.instance, args.gpu) != (expected["instance_id"], expected["gpu"]):
        raise ValueError("instance/GPU not authorized for this lane")
    physical_source = is_device_source(source)
    from gripper_distance_plan import NATIVE_DISTANCE_ENCODING
    if plan.get('native_action_encoding') == NATIVE_DISTANCE_ENCODING and (
            physical_source or plan.get('planner_id') != CUROBO_PLANNER_ID or getattr(args, 'force_diagnostic_replay', False)):
        raise ValueError('dual simulation distance replay requires its explicit MotionGen plan')
    binding_path = getattr(args, "hardware_binding", None)
    independent = getattr(args, "independent_replay", False)
    if physical_source:
        if (args.side != "dsw-scene" or getattr(args,"mode",None) != "replay" or not independent
                or not binding_path or getattr(args,"force_diagnostic_replay",False) or getattr(args,"barrier",None)):
            raise ValueError("physical source permits only independent validated target simulation replay")
        from real_contract import validate_real_plan
        validate_real_plan(plan, source, json.loads(Path(binding_path).read_text()),environment=getattr(args,'environment','physical'))
    else:
        if binding_path:
            raise ValueError("physical binding cannot use simulated source")
        bridge = DualGpuPipeline()
        bridge.ingest_source(source)
        bridge.accept_plan(plan)
    from joint_space import validate as validate_joint_space
    validate_joint_space(plan, source)
    episodes = suite.get("episodes") if isinstance(suite, dict) else None
    if not isinstance(episodes, list) or len(episodes) != 1 or not isinstance(episodes[0], dict):
        raise ValueError("replay requires exactly one actual episode scene")
    keys = ("episode_id", "seed", "machine_asset", "environment_from_machine_root", "environment_from_cup")
    if any(key not in episodes[0] for key in keys):
        raise ValueError("actual episode is missing physical scene fields")
    actual = {key: episodes[0][key] for key in keys}
    if args.side == "dsw-scene" and actual != source["scene"]["episode_scene"]:
        raise ValueError("target actual suite must match the detected episode_scene")
    if 'initial_state' in source:
        model = source['initial_state']['model']
        metadata = episodes[0].get('metadata') or {}
        if metadata.get('initial_robot_state') != model or metadata.get('initial_robot_state_digest') != _digest(model):
            raise ValueError('replay suite must preserve sealed model initial state')
    if args.side == "dsw-scene" and episodes[0].get("randomization") is not None:
        raise ValueError("target cannot randomize the frozen detected scene")
    if not (len(plan.get("native_actions", [])) == len(plan.get("native_substeps", [])) == len(plan["ticks"])):
        raise ValueError("native actions and substeps must cover every frozen tick")
    return {"actual_episode_scene": actual, "actual_episode_digest": _digest(actual),
            "detected_scene_digest": source["scene"]["scene_digest"]}


def configure_whitebox_fast_scene(cfg, *, width=640, height=480):
    """Keep task geometry, simple lighting and one readable RGB monitor camera."""
    for name in ("room", "d435_model", "handoff_stand"):
        if hasattr(cfg.scene, name):
            setattr(cfg.scene, name, None)
    camera = getattr(cfg.scene, "workspace_camera", None)
    if camera is None:
        raise ValueError("whitebox_fast requires the workspace camera")
    camera.width, camera.height = width, height
    camera.data_types = ["rgb"]
    camera.update_period = 1 / 30
    return cfg


def apply_whitebox_materials():
    """Matte, untextured task meshes; preserve surface shading and object contrast."""
    import isaaclab.sim as sim_utils
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    stage = sim_utils.get_current_stage()
    roots = ("/World/envs/env_0/Robot", "/World/envs/env_0/Worktable",
             "/World/envs/env_0/CoffeeMachine", "/World/envs/env_0/Cup")
    colors = ((.62, .67, .73), (.30, .34, .39), (.72, .75, .78), (.82, .65, .40))
    bound = 0
    for root, color in zip(roots, colors):
        path = "/World/PreviewMaterials/" + root.rsplit("/", 1)[-1]
        material = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.85)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        prim = stage.GetPrimAtPath(root)
        if not prim:
            continue
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdGeom.Gprim):
                UsdShade.MaterialBindingAPI.Apply(child).Bind(material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
                bound += 1
    if not bound:
        raise ValueError("whitebox_fast found no renderable geometry")
    return {"material": "/World/PreviewMaterials", "bound_prims": bound, "roots": roots,
            "camera": "d405_base_0_pose", "resolution": [640, 480], "style": "matte_mesh"}


def make_software_encoder(work):
    """Use native VideoWriter's ffmpeg hook; cap each software encoder's worker pool."""
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise ValueError("recording requires ffmpeg with the libx265 software encoder")
    path = Path(work) / "ffmpeg-cpu-limited"
    # The system Python avoids Isaac's library path, which the native writer intentionally clears.
    source = ("#!/usr/bin/python3\nimport os, sys\n"
              f"os.execv({ffmpeg!r}, [{ffmpeg!r}, *sys.argv[1:-1], "
              "'-threads', '2', '-x265-params', 'pools=2:frame-threads=1', sys.argv[-1]])\n")
    with path.open("x") as stream:
        stream.write(source)
    path.chmod(0o700)
    return path


def raw_job(args, plan, source, episode_id, task_id, prompt, source_files):
    """Build the existing CoffeeCapture job, with honest frozen-replay provenance."""
    if plan["control"]["physics_hz"] != 240 or plan["control"]["substeps"] != 8:
        raise ValueError("native raw writer requires 240 Hz physics and 8 substeps per control tick")
    substeps = plan["native_substeps"]
    if (len(substeps) != len(plan["ticks"]) or not substeps or
            any(type(n) is not int or not 1 <= n <= 8 for n in substeps) or any(n != 8 for n in substeps[:-1])):
        raise ValueError("only the final frozen tick may contain fewer than 8 substeps")
    if _digest(source["calibration_profile"]) != plan["calibration_profile_digest"]:
        raise ValueError("raw calibration profile differs from frozen plan")
    replay = {key: plan[key] for key in ("run_id", "source_run_id", "source_sequence", "scene_digest",
                                        "timeline_digest", "calibration_profile_digest")}
    replay.update(instance_id=args.instance, gpu=args.gpu, side=args.side,
                  observation_adapter=source["adapter"], scheduling=replay_scheduling(args), evidence_source="simulation")
    generation = {"id": "coffee_frozen_timeline_replay", "version": "1", "kind": "frozen_timeline",
                  "parameters_requested": replay,
                  "parameters_resolved": {"motion_profile": plan["motion_profile"], "control": plan["control"],
                                          "episode_id": episode_id, "tick_count": len(plan["ticks"])}}
    generation["sha256"] = raw_digest(generation)
    return {"directory": str(args.output / "raw"), "asset_root": str(args.assets),
            "batch": {"task_id": task_id, "batch_id": args.run_id, "prompt": prompt,
                      "prompt_sha256": raw_digest(prompt),
                      "attempts": [{"episode_id": episode_id, "request_id": args.run_id + ":" + args.side}]},
            "generation_plan": generation,
            # Native VideoWriter has no explicit NVENC GPU selector. Software encoding cannot touch GPU0.
            "encoder_options": {"encoder": "libx265", "quality": 14, "timeout": 60},
            "provenance": {"commit": plan.get("source_revision"), "source_kind": "frozen_timeline_replay",
                           "source_files": source_files, "source_snapshot_sha256": raw_digest(source_files),
                           "replay": replay, "prompt_file_sha256": file_sha256(args.raw_prompt_file)}}


def make_raw_capture(args, plan, source, episode, env, prompt):
    from importlib.metadata import PackageNotFoundError, version
    from robot_sim.collection.coffee_capture import CoffeeCapture

    native_root = Path(inspect.getfile(CoffeeCapture)).resolve().parents[4]
    paths = sorted((native_root / "source/robot_sim/robot_sim").rglob("*.py"))
    source_files = {str(p.relative_to(native_root)): file_sha256(p) for p in paths}
    source_files["workbench/isaac_lane.py"] = file_sha256(Path(__file__))
    for name in ("contract.py", "integration.py", "gpu_acceptance.py", "curobo_planning.py", "a1_support_domain.json", "a1_support_near.json"):
        source_files["workbench/" + name] = file_sha256(Path(__file__).with_name(name))
    versions = {"python": sys.version}
    for name in ("numpy", "torch", "isaaclab", "isaacsim"):
        try:
            versions[name] = version(name)
        except PackageNotFoundError:
            versions[name] = None
    spool=Path('/tmp')/('rbw-raw-'+hashlib.sha256(str(args.output).encode()).hexdigest()[:20])
    if shutil.disk_usage(spool.parent).free<2*1024**3:raise ValueError('raw local spool requires 2GiB free')
    spool.mkdir(mode=0o700,exist_ok=False)
    work = spool / ".work"
    work.mkdir(parents=True, exist_ok=False)
    encoder = make_software_encoder(work)
    source_files["workbench/ffmpeg-cpu-limited"] = file_sha256(encoder)
    job = raw_job(args, plan, source, episode.episode_id, env.cfg.generalization.task_id, prompt, source_files)
    job['directory']=str(spool)
    job['provenance']['recording_storage']=dict(mode='local-disk-spool-then-verified-cpfs',spool_root=str(spool),publish_root=str(args.output/'raw'))
    replay_policy = getattr(env._m2_runtime, '_motiongen_replay_policy', None)
    if replay_policy is not None:
        job['provenance']['execution_policy'] = deepcopy(replay_policy['configuration'])
        job['provenance']['execution_policy_sha256'] = replay_policy['configuration_sha256']
    job["encoder_options"]["ffmpeg"] = str(encoder)
    job["provenance"]["encoder_cpu_limits"] = {"worker_pool": 2, "frame_threads": 1, "ffmpeg_threads": 2}
    write(work / "job.json", job)
    identity = {"physical_configuration": env._m2_runtime.physical_identity,
                "physical_configuration_sha256": env._m2_runtime.physical_sha256,
                "planner_id": plan["planner_id"], "runtime_versions": versions,
                "motion_profile": plan["motion_profile"], "control": plan["control"],
                "replay": job["provenance"]["replay"]}
    if replay_policy is not None:
        identity['execution_policy'] = deepcopy(replay_policy['configuration'])
        identity['execution_policy_sha256'] = replay_policy['configuration_sha256']
    capture = CoffeeCapture(job, episode, env, identity)
    capture._workbench_spool=spool
    capture._workbench_publish=args.output/'raw'
    write(capture.record.root / "sim/frozen_plan.json", plan)
    write(capture.record.root / "sim/source_observation.json", source)
    return capture


def publish_raw_spool(capture):
    """Copy a closed raw tree, verify every byte, then publish by atomic rename."""
    spool=getattr(capture,'_workbench_spool',None)
    if spool is None:return Path(capture.record.root)
    destination=capture._workbench_publish;staging=destination.with_name(destination.name+'.partial')
    files=[p for p in spool.rglob('*') if p.is_file()]
    if any(p.is_symlink() for p in spool.rglob('*')):raise ValueError('raw spool may not contain symlinks')
    if sum(p.stat().st_size for p in files)>2*1024**3:raise ValueError('raw spool exceeds 2GiB budget')
    if destination.exists() or staging.exists():raise FileExistsError('raw publication already exists')
    relative=Path(capture.record.root).relative_to(spool)
    shutil.copytree(spool,staging)
    if any(file_sha256(p)!=file_sha256(staging/p.relative_to(spool)) for p in files):
        raise ValueError('raw publication copy hash differs')
    for p in files:
        with (staging/p.relative_to(spool)).open('rb') as stream:os.fsync(stream.fileno())
    staging.rename(destination)
    directory=os.open(destination.parent,os.O_RDONLY)
    try:os.fsync(directory)
    finally:os.close(directory)
    return destination/relative


def raw_recording_summary(root, plan):
    """Only a sealed native manifest and exact cadence authorize a complete summary."""
    root = Path(root)
    manifest_path = root / "manifests/episode.json"
    manifest = json.loads(manifest_path.read_text())
    final_step = sum(plan["native_substeps"])
    if (manifest.get("schema_version") != "sim_raw_dataset/v1" or manifest.get("capture_status") != "complete"
            or manifest.get("validation", {}).get("status") != "passed" or manifest.get("outcome") != "success"
            or manifest.get("final_physics_step") != final_step):
        raise ValueError("raw episode was not sealed successfully through the frozen final physics step")
    metadata = json.loads((root / "camera/metadata.json").read_text())
    counts = {device: metadata[device + "_rgb"]["frames"] for device in ("base_0", "left_wrist_0", "right_wrist_0")}
    if any(type(count) is not int or count != final_step // 8 + 1 for count in counts.values()):
        raise ValueError("raw camera frames do not cover the native regular-samples-through-K boundary")
    return {"enabled": True, "capture_status": "complete", "schema_version": manifest["schema_version"],
            "directory": str(root), "manifest": str(manifest_path), "manifest_sha256": file_sha256(manifest_path),
            "validation": "passed", "outcome": manifest["outcome"], "final_physics_step": final_step,
            "native_state_samples": final_step + 1, "robot_samples_per_arm": final_step // 2 + 1,
            "camera_frames": counts, "timeline_digest": plan["timeline_digest"],
            "encoder": "libx265", "clock": "synthetic_simulation", "utc_evidence": "execution.jsonl"}


def recorded_step(env, capture, request, stage, tick, timeline_digest):
    if capture:
        capture.before_command(SimpleNamespace(request=request, stage=stage))
        capture.record.active_command.update(source_kind="frozen_timeline_replay",
                                             timeline_digest=timeline_digest, timeline_tick=tick)
    try:
        return env.step(request)
    finally:
        if capture:
            capture.record.end_command()


def close_failed_recording(capture, info, error):
    failure = {"capture_status": "failed", "directory": str(capture.record.root)}
    if not capture.record.finished:
        try:
            capture.finish("execution_failed", info, str(error))
        except Exception:
            failure["finalization_error"] = traceback.format_exc()
    return failure


def publish_failure(args, error):
    # Notify the peer before raw finalization or simulator shutdown can block.
    failure = {"status": "failed", "error": str(error), "run_id": args.run_id}
    write(args.output / "failure.json", failure)
    if args.mode == "replay" and args.barrier and not getattr(args,"independent_replay",False):
        write(args.barrier / (args.side + "-failed.json"), failure)


def close_runtime(env, app):
    # SimulationApp.close() may raise SystemExit(0); never mask the original failure.
    for resource in (env, app):
        if resource is not None:
            try:
                resource.close()
            except SystemExit:
                pass
            except Exception:
                traceback.print_exc()


def run(args):
    prompt = validate_record_options(args)
    if getattr(args,'rebuild_finger_envelope',False) and args.mode!='geometry-inputs':
        raise ValueError('finger envelope rebuilding is offline geometry preparation only')
    profile_limit=getattr(args,'profile_stop_after_ticks',None)
    if profile_limit is not None and (args.mode!='replay' or not args.record_raw or args.hardware_binding or not 1<=profile_limit<=60):
        raise ValueError('profile stop requires a simulation raw replay of 1..60 ticks')
    if not math.isfinite(args.gripper_opening_mm) or not 0 < args.gripper_opening_mm <= 88:
        raise ValueError("gripper opening must be finite and in (0, 88] mm")
    if args.mode == "replay" and args.gripper_opening_mm != 88:
        raise ValueError("replay gripper opening comes only from the frozen plan")
    suite = json.loads(args.suite.read_text())
    plan = json.loads(args.plan.read_text()) if args.mode == "replay" else None
    source = json.loads(args.source.read_text()) if plan or args.mode == 'geometry-inputs' else None
    actual_scene = validate_replay_inputs(args, plan, source, suite) if plan else None
    if args.mode == 'geometry-inputs':
        validate_geometry_inputs(args, source, suite)
    from gripper_distance_plan import NATIVE_DISTANCE_ENCODING
    distance = bool(plan and plan.get('native_action_encoding') == NATIVE_DISTANCE_ENCODING)
    os.environ["ROBOT_SIM_ASSET_ROOT"] = str(args.assets)
    # CUDA stays masked; Vulkan uses the physical ordinal and disables automatic multi-GPU.
    device = "cuda:0"
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True, device=device,
                      active_gpu=args.gpu, multi_gpu=False,
                      kit_args=f"--/renderer/activeGpu={args.gpu} --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false --/renderer/gpuEnumeration/glInterop/enabled=false").app
    env, capture, info = None, None, None
    replay_started = None
    step_seconds = barrier_seconds = 0.0
    applied = []
    try:
        import numpy as np
        from robot_sim.tasks.manager_based.coffee_make.env import CoffeeMakeEnv
        from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
        from robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg import make_generalization_env_cfg
        from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec, ActionRequest
        from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import EpisodeAcceptanceSpec
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
        from robot_sim.collection.arx_state import ArxState

        control = ControlSpec(**plan["control"]) if plan else ControlSpec(240, 8, 2.0, 5.0, 300.0)
        opening_mm = plan.get("gripper_opening_mm", 88.0) if plan else args.gripper_opening_mm
        acceptance = replace(EpisodeAcceptanceSpec.development(), open_position_m=opening_mm / 2000)
        episode = EpisodeScene.from_dict(suite["episodes"][0])
        if args.whitebox_fast and args.mode != "replay":
            raise ValueError("whitebox_fast is replay-only; observation still needs the D405 RGB-D rig")
        cfg = make_generalization_env_cfg(control=control,
            acceptance=acceptance, asset_root=args.assets,
            cameras=True, camera_rig=None if args.whitebox_fast else "d405",
            physics_device=device, pad_torsional_radius_m=0.0015, solver_profile="contact_32_8",
            episode=episode, continuous_motion=True)
        if args.whitebox_fast:
            configure_whitebox_fast_scene(cfg)
            from robot_sim.asset_cfg.d405 import camera_offset, load_rig
            rig = load_rig(args.assets)
            head = next(c for c in rig["cameras"] if c["device_calibration_ref"] == "base_0")
            camera = cfg.scene.workspace_camera
            camera.prim_path = f"{{ENV_REGEX_NS}}/Robot/{head['parent']}/PreviewCamera"
            camera.offset.pos, camera.offset.rot = camera_offset(head, rig)
            camera.spawn.horizontal_aperture = 24 * 800 / 400
            camera.spawn.clipping_range = (.01, 5.0)
        if args.mode == "observe":
            configure_observation_scene(cfg)
        env = CoffeeMakeEnv(cfg)
        whitebox_material = apply_whitebox_materials() if args.whitebox_fast else None
        runtime = env._m2_runtime
        cup_preset=plan.get('cup_contact_preset') if plan else None
        if cup_preset:
            if (args.mode!='replay' or not distance or args.hardware_binding or args.force_diagnostic_replay
                    or source.get('lane')!='simulated-real'):
                raise ValueError('cup preset characterization is dual simulation only')
            from cup_presets import configure,bind_preset_identity
            bind_preset_identity(runtime.physical_identity,cup_preset)
            import isaaclab.sim as sim_utils
            preset_material=configure(sim_utils.get_current_stage(),runtime.root,cup_preset['stiffness_n_m'],cup_preset['damping_n_s_m'])
            write(args.output/'cup-contact-preset.json',dict(preset=cup_preset,material=preset_material,default_selected=False,scope='simulation diagnosis only'))
        if distance or any(getattr(b,'radius_start_m',None) is not None for b in runtime.cup_profile.bands):
            install_grasp_contact_adapter(runtime,diagnostic_compression=bool(cup_preset))
        bind_effective_gripper_identity(runtime, acceptance.open_position_m,
                                        env.action_manager.get_term("guarded").gripper_limits())
        diagnostic_events = enable_forced_diagnostic_replay(runtime) if args.force_diagnostic_replay else []
        if plan:
            validate_physical_identity(runtime, plan)
        _, info = env.reset(options={"episode": episode})
        if args.mode == "observe":
            validate_observation_outputs(env.scene)
            if any(getattr(b,'radius_start_m',None) is not None for b in runtime.cup_profile.bands):
                from gpu_acceptance import bind_runtime_support_gpu
                from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
                support_policy=bind_runtime_support_gpu(runtime,ac_one_asset_spec(args.assets).urdf_path)
                write(args.output/'runtime-support-policy.json',support_policy)
        spec = ac_one_dual_arm_zero_home_action_spec()
        robot = env.scene["robot"]
        projection = ArxState(robot.joint_names)

        def measured():
            x = projection.project(robot.data.joint_pos[0].cpu().numpy(), robot.data.joint_vel[0].cpu().numpy())
            result = demo_state()
            for side in ("left", "right"):
                result[side + "_arm_joints"] = x[side]["joint"][:6].astype(float).tolist()
                result[side + "_gripper"] = max(0.0, float(x[side]["joint"][6]) * 1000)
            return result

        sample, measured_scene = runtime.read(runtime.loop.physics_steps)
        reset_scene = {"environment_from_machine_root": asdict(measured_scene.world_from_machine),
                       "environment_from_cup": asdict(sample.world_from_cup)}
        if actual_scene is not None:
            actual_scene.update(measured_reset_scene=reset_scene, measured_scene_source="isaac_post_reset_mesh_roots")
        from joint_space import replay_state
        physical_reset = args.mode == "replay" and is_device_source(source)
        reset_state = replay_state(measured(), source) if physical_reset else measured()
        write(args.output / "reset.json", {"episode": asdict(episode), "state": reset_state, "info": info,
                                           "joint_space": "reported" if physical_reset else "model",
                                           **({} if physical_reset else {'simulation_initial_command': simulation_initial_command(
                                               runtime.loop.previous, spec, opening_mm, runtime.loop.physics_steps)}),
                                           "measured_reset_scene": reset_scene})
        validate_reset_grippers(measured(),episode,opening_mm)
        if distance:
            spec = install_distance_replay(env, runtime, spec, plan)
            replay_policy = install_motiongen_replay_policy(runtime, plan)
            write(args.output/'execution-policy.json', replay_policy)
            if plan['planner_id']==CUROBO_PLANNER_ID:
                from gpu_acceptance import bind_runtime_support_gpu
                from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
                support_policy=bind_runtime_support_gpu(runtime,ac_one_asset_spec(args.assets).urdf_path)
                write(args.output/'runtime-support-policy.json',support_policy)
        from PIL import Image
        sensors = preview_sensors(env.scene, whitebox_fast=args.whitebox_fast)
        preview_sensor = sensors["base_0"]
        pixels=preview_sensor.data.output['rgb'][0,...,:3] if args.whitebox_fast else calibrated_camera_rgb(env,'base_0')
        Image.fromarray(pixels.cpu().numpy()).save(args.output / "camera.png")
        # One bounded latest-frame cache per existing camera is shared with the UI.
        # The replay/observation loop publishes after every control tick; readers
        # may drop frames without creating a stale queue or per-frame SSH call.
        # Wrist caches are disposable; atomic replace suffices without extra CPFS fsync barriers.
        previews = {camera: PreviewPublisher(camera_frame_path(args.output, camera), durable=False)
                    for camera in sensors}
        publish_previews(previews, sensors,
                         {"lane": args.side, "mode": args.mode, "tick": 0, "stage": "reset",
                          **(replay_provenance(plan) if plan else {})},
                         whitebox_fast=args.whitebox_fast,env=env)
        if args.mode == 'geometry-inputs':
            export_geometry_inputs(args, source, runtime, episode, control, acceptance, info, measured())
        elif args.mode == "observe":
            from scipy.spatial.transform import Rotation
            camera = env.scene["d405_base_0"].data
            camera_xyz = camera.pos_w[0].cpu().numpy().astype(float).tolist()
            camera_q = camera.quat_w_ros[0].cpu().numpy().astype(float).tolist()
            camera_rpy = Rotation.from_quat(camera_q[1:] + camera_q[:1]).as_euler("xyz").tolist()
            profile = {"schema_version": "calibration_profile.v1", "profile_id": "isaac-d405-base0-v2",
                "source": {"kind": "simulation-authored", "hardware_calibrated": False},
                "camera": {"xyz_m": camera_xyz, "rpy_rad": camera_rpy,
                           "convention": "world_from_ros_optical", "frame_id": "d405_base_0_color_optical_frame",
                           "K": calibrated_camera_k(env,'base_0'),"image_domain":"calibrated_color",
                           "width":640,"height":480},
                "joints": {"zero_offsets_rad": [0]*12, "signs": [1]*12},
                "gripper": {"mapping": "sum_of_two_prismatic_displacements_mm", "max_opening_mm": opening_mm},
                "frames": {"environment_from_camera": {"position": camera_xyz, "orientation_wxyz": camera_q}},
                "divergence_thresholds": {"joint_rad": 0.05, "gripper_mm": 5, "timing_s": 0.1},
                "threshold_provenance": "simulation-development thresholds; not hardware measurements"}
            profile["profile_id"] += "-" + _digest(profile)[:12]
            if episode.machine_asset == "objects/owner_coffee_machine_00003/v1":
                # Installation geometry, never the measured object reset poses.
                from robot_sim.tasks.manager_based.coffee_make.layout import coffee_episode_layout
                layout = coffee_episode_layout()
                table_z = layout.worktable.position[2] + layout.worktable_size_m[2] / 2
                profile["scene_support"] = {"machine_asset": episode.machine_asset,
                    "machine_root_z_m": table_z, "cup_root_z_m": table_z,
                    "source": "coffee_episode_layout/table_top; canonical upright mesh roots at base"}
                profile["profile_id"] += "-support-" + _digest(profile["scene_support"])[:12]
            window = ObservationWindow(profile, [], coordinates="world")
            snapshots = []
            captures = []
            observation_steps = round(args.observation_seconds * 30)
            for i in range(observation_steps + 1):
                sample, scene = runtime.read(runtime.loop.physics_steps)
                pose = asdict(scene.world_from_machine)
                q = pose["orientation_wxyz"]
                snapshot = {"mode": "live", "source_stamp_ns": 1_000_000_000 + round(i * 1e9 / 30),
                    "frame_id": "environment", "tracks": [
                        {"object_type": "coffee_machine", "pose_current": True, "xyz_m": pose["position"],
                         "quaternion_xyzw": [q[1], q[2], q[3], q[0]]},
                        {"object_type": "cup", "pose_current": True, "xyz_m": list(sample.world_from_cup.position)}]}
                for track in snapshot["tracks"]:
                    track["world"] = {key: track[key] for key in ("xyz_m", "quaternion_xyzw") if key in track}
                window.add(snapshot)
                snapshots.append(snapshot)
                publish_previews(previews, sensors,
                                 {"lane": args.side, "mode": "observe", "tick": i, "stage": "observe",
                                  "source_stamp_ns": snapshot["source_stamp_ns"]},env=env)
                if i % 30 == 0:
                    data = env.scene["d405_base_0"].data
                    name = f"rgbd-{i:04d}.npz"
                    np.savez_compressed(args.output / name, rgb=calibrated_camera_rgb(env,'base_0').cpu().numpy(),
                        depth_m=calibrated_camera_depth(env,'base_0').cpu().numpy().astype(np.float32),
                        K=np.asarray(calibrated_camera_k(env,'base_0')), position=data.pos_w[0].cpu().numpy(),
                        quaternion_wxyz=data.quat_w_ros[0].cpu().numpy())
                    captures.append({"path": name, "source_stamp_ns": snapshot["source_stamp_ns"], "capture_sequence_id": i})
                if i < observation_steps:
                    _, _, _, _, info = env.step(ActionRequest(info["episode_id"], info["step_id"], runtime.loop.previous))
            result = window.finalize()
            result["source"]["machine_orientation"] = "Isaac measured mesh root; ground truth transport test"
            result["source"]["adapter"] = "sim-online-detection-ground-truth"
            result.pop("scene_digest")
            result["scene_digest"] = _digest(result)
            message = {"schema_version": "dual_gpu_pipeline.v1", "gpu": args.gpu, "instance_id": args.instance,
                "lane": args.side, "run_id": args.run_id, "sequence": 0, "source_stamp_ns": snapshots[-1]["source_stamp_ns"],
                "adapter": "sim-online-detection-ground-truth", "scene": result,
                "observation_window": window.status(), "calibration_profile": profile,
                "transport": {"kind": "shared-cpfs-atomic-json"}}
            write(args.output / "source.json", message)
            write(args.output / "profile.json", profile)
            write(args.output / "snapshots.json", snapshots)
            write(args.output / "rgbd-manifest.json", {"frame_id": "d405_base_0_color_optical_frame",
                "source_clock": "simulation", "capture_hz": 1, "control_hz": 30,
                "depth_convention": "optical_z_m", "camera_pose_convention": "world_from_ros_optical", "frames": captures})
            write(args.output / "suite.json", {"episodes": [result["episode_scene"]]})
            for name in env.scene.sensors:
                if not name.startswith("d405_"):
                    continue
                sensor = env.scene[name]
                data = sensor.data
                device=name.removeprefix('d405_');pixels=calibrated_camera_rgb(env,device).cpu().numpy()
                np.savez_compressed(args.output / (name + ".npz"), rgb=pixels,
                    depth_m=calibrated_camera_depth(env,device).cpu().numpy(),
                    K=np.asarray(calibrated_camera_k(env,device)), position=data.pos_w[0].cpu().numpy(),
                    quaternion_wxyz=data.quat_w_ros[0].cpu().numpy())
                from PIL import Image
                Image.fromarray(pixels).save(args.output / (name + ".png"))
            write(args.output / "final.json", {"status": "observed", "window": window.status(),
                "perception_validated": False, "scene_digest": result["scene_digest"],
                "live_preview": preview_stats(previews), "whitebox_fast": whitebox_material})
        else:
            if _digest(plan["ticks"]) != plan["timeline_digest"]:
                raise ValueError("timeline digest mismatch")
            if plan["scene_digest"] != source["scene"]["scene_digest"]:
                raise ValueError("scene digest mismatch")
            recording = {"enabled": False, "capture_status": "not_requested"}
            if args.record_raw:
                capture = make_raw_capture(args, plan, source, episode, env, prompt)
                capture.start()
                if getattr(capture,'profile',None) and 'support_policy' in locals():
                    capture.profile.wrap(runtime.zero_collision.support_contact,'check','gpu_support_check')
            replay_handshake(args)
            applied, states, stamps = [], [], []
            from joint_space import replay_state
            replay_started = time.perf_counter()
            preview_updated=time.monotonic()
            step_seconds = barrier_seconds = 0.0
            telemetry_path = args.output / "telemetry.jsonl"
            with (args.output / "execution.jsonl").open("x") as log, telemetry_path.open("x") as telemetry:
                for tick, raw, expected_substeps in zip(plan["ticks"], plan["native_actions"], plan["native_substeps"], strict=True):
                    if replay_state(native_target(raw, spec, opening_mm, distance=distance), source) != tick["target"]:
                        raise ValueError("native action differs from physical target")
                    n = tick["tick"]
                    # The initial handshake or previous completion already proves both peers are ready.
                    stamp = time.time_ns()
                    request = ActionRequest(info["episode_id"], info["step_id"], tuple(raw))
                    stage = next((item["name"] for item in reversed(plan.get("stages", []))
                                  if item["start_tick"] <= n), "frozen_replay")
                    if distance:
                        spec.set_stage(stage)
                    diagnostic_count = len(diagnostic_events)
                    stepping = time.perf_counter()
                    _, _, _, _, info = recorded_step(env, capture, request, stage, n, plan["timeline_digest"])
                    step_seconds += time.perf_counter() - stepping
                    report = info["execution_report"]
                    state = replay_state(measured(), source)
                    if args.side=='simulated-real' and time.monotonic()-preview_updated>=1.:
                        publish_previews(previews, sensors,
                                     {"lane": args.side, "mode": "replay", "tick": n, "stage": stage,
                                      "source_stamp_ns": stamp, **replay_provenance(plan)}, whitebox_fast=args.whitebox_fast,env=env)
                        preview_updated=time.monotonic()
                    row = replay_telemetry(runtime, stage, tick, tick["target"], report, state, distance=distance)
                    if distance:
                        row['execution_policy_sha256'] = replay_policy['configuration_sha256']
                        row['frozen_success_progress'] = deepcopy(replay_policy['progress'])
                    row["diagnostic_events"] = diagnostic_events[diagnostic_count:]
                    telemetry.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                    telemetry.flush()
                    if (not report["action_applied"] or report["actual_substeps"] != expected_substeps
                            or (report['terminated'] and (report.get('event') or {}).get('reason')!='success')
                            or replay_state(native_target(report["applied_action"], spec, opening_mm, distance=distance), source) != tick["target"]):
                        raise RuntimeError(f"command rejected at tick {n}: {report}")
                    applied.append(tick)
                    states.append(state)
                    stamps.append(stamp)
                    log.write(json.dumps({"tick": n, "applied": tick, "measured": state, "utc_ns": stamp,
                                          "execution_report": report}) + "\n")
                    log.flush()
                    if n%30==0 or (profile_limit is not None and n+1>=profile_limit):
                        probe=runtime_profile_snapshot(capture,n+1)
                        if probe is not None:write(args.output/f'runtime-profile-{n:06d}.json',probe)
                    if profile_limit is not None and n+1>=profile_limit:
                        raise RuntimeError(f'diagnostic profiling stop after {profile_limit} ticks; partial recording is not an accepted replay')
                    waiting = time.perf_counter()
                    replay_tick_sync(args, n)
                    if not getattr(args,"independent_replay",False):
                        barrier_seconds += time.perf_counter() - waiting
            elapsed = time.perf_counter() - replay_started
            pixels=preview_sensor.data.output['rgb'][0,...,:3] if args.whitebox_fast else calibrated_camera_rgb(env,'base_0')
            Image.fromarray(pixels.cpu().numpy()).save(args.output / "final-camera.png")
            performance = {"clock": "local_monotonic", "execution_wall_s": elapsed,
                "simulation_s": sum(plan["native_substeps"]) / plan["control"]["physics_hz"],
                "native_step_and_capture_s": step_seconds, "barrier_io_and_wait_s": barrier_seconds,
                "other_s": max(0.0, elapsed - step_seconds - barrier_seconds),
                "live_preview": preview_stats(previews), "whitebox_fast": whitebox_material}
            write(args.output / "performance.json", performance)
            terminal_reason = (info["execution_report"].get("event") or {}).get("reason")
            if distance:
                finish_motiongen_replay_policy(runtime)
                write(args.output/'execution-policy-final.json', replay_policy)
            if terminal_reason not in (None, "success"):
                raise RuntimeError(f"replay ended with non-diagnostic terminal event: {terminal_reason}")
            if args.force_diagnostic_replay and len(applied) != len(plan["ticks"]):
                raise RuntimeError("forced diagnostic replay stopped before the frozen timeline completed")
            if terminal_reason != "success" and not args.force_diagnostic_replay:
                raise RuntimeError("replay ended without native task success")
            if capture:
                capture.finish("success", info)
                recording = raw_recording_summary(publish_raw_spool(capture), plan)
            status = "diagnostic-complete" if args.force_diagnostic_replay else "completed"
            write(args.output / "execution.json", {"schema_version": "dual_gpu_pipeline.v1", "side": args.side,
                "instance_id": args.instance, "gpu": args.gpu, "run_id": args.run_id,
                "timeline_digest": plan["timeline_digest"], "tick_count": len(applied), "status": status,
                "source_run_id": plan["source_run_id"], "source_sequence": plan["source_sequence"],
                "scene_digest": plan["scene_digest"], "calibration_profile_digest": plan["calibration_profile_digest"],
                "plan_run_id": plan["run_id"],
                "applied_ticks": applied, "measured_states": states, "applied_at_utc_ns": stamps,
                "clock_uncertainty_s": None, "scheduling": replay_scheduling(args),
                "evidence_source": "simulation",
                "final_info": info, "recording": recording, "performance": performance,
                "execution_policy": deepcopy(replay_policy) if distance else None,
                "diagnostic": {"enabled": bool(args.force_diagnostic_replay),
                               "collision_events": diagnostic_events,
                               "terminal_reason": terminal_reason}, **actual_scene})
            write(args.output / "final.json", {"status": "diagnostic-complete" if args.force_diagnostic_replay else "replay-complete",
                                                "ticks": len(applied), "recording": recording,
                                                "scheduling": replay_scheduling(args), **replay_provenance(plan),
                                                "execution_policy": deepcopy(replay_policy) if distance else None,
                                                "collision_events": diagnostic_events})
    except Exception as error:
        publish_failure(args, error)
        if replay_started is not None and args.mode == "replay":
            try:
                elapsed = time.perf_counter() - replay_started
                write(args.output / "performance.json", {
                    "clock": "local_monotonic", "status": "failed",
                    "execution_wall_s": elapsed,
                    "simulation_s": sum(plan["native_substeps"]) / plan["control"]["physics_hz"],
                    "completed_ticks": len(applied),
                    "native_step_and_capture_s": step_seconds,
                    "barrier_io_and_wait_s": barrier_seconds,
                    "other_s": max(0.0, elapsed - step_seconds - barrier_seconds),
                    "live_preview": preview_stats(previews) if "previews" in locals() else None,
                    "whitebox_fast": whitebox_material if "whitebox_material" in locals() else None,
                    "error": str(error),
                })
            except Exception:
                pass
        failure = {"status": "failed", "error": traceback.format_exc()}
        if "runtime" in locals() and hasattr(runtime, '_motiongen_replay_policy'):
            failure['execution_policy'] = deepcopy(runtime._motiongen_replay_policy)
        if "runtime" in locals() and getattr(runtime, "trace", None):
            failure["last_physics_trace"] = runtime.trace[-1]
        # Preserve the cause before encoding/publishing a potentially large raw.
        write(args.output / "failure-physics.json", failure)
        if capture:
            failure["recording"] = close_failed_recording(capture, info, error)
            try:failure['recording']['published_directory']=str(publish_raw_spool(capture))
            except Exception as publication_error:failure['recording']['publication_error']=str(publication_error)
        write(args.output / "failure-detail.json", failure)
        print(failure["error"], flush=True)
        raise
    finally:
        close_runtime(env, app)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=["observe", "geometry-inputs", "replay"], required=True)
    p.add_argument("--gpu", type=int, required=True)
    p.add_argument("--instance", required=True)
    p.add_argument("--side", choices=["simulated-real", "dsw-scene"], required=True)
    p.add_argument("--run-id", required=True)
    p.add_argument("--gripper-opening-mm", type=float, default=88.0, help="observe-only reset/profile opening; replay uses its frozen plan")
    p.add_argument("--record-raw", action="store_true", help="record native three-camera raw episode using CPU libx265")
    p.add_argument('--profile-stop-after-ticks',type=int,help='simulation-only profiling; deliberately fails after 1..60 ticks and seals partial evidence')
    p.add_argument("--force-diagnostic-replay", action="store_true",
                   help="simulation-only replay: log collision events and continue; never a successful dataset")
    p.add_argument("--hardware-binding", type=Path, help="Trusted physical calibration/source receipt for target-only replay")
    p.add_argument('--environment',choices=('physical','fake'),default='physical')
    p.add_argument('--execution-json')
    p.add_argument("--independent-replay", action="store_true", help="Physical target simulation runs without peer barriers")
    p.add_argument("--whitebox-fast", action="store_true",
                   help="replay with matte task meshes and a 640x480 top-camera monitor")
    p.add_argument("--raw-prompt-file", type=Path, help="required with --record-raw: JSON text/language/version")
    p.add_argument("--observation-seconds", type=int, default=5, choices=range(5, 61))
    p.add_argument('--finger-envelope', type=Path)
    p.add_argument('--finger-envelope-sha256')
    p.add_argument('--rebuild-finger-envelope',action='store_true',help='offline geometry-inputs only: rebuild the native vertex/plane envelope for a changed physics profile')
    for field in ("suite", "assets", "output", "plan", "source", "barrier"):
        p.add_argument("--" + field, type=Path, required=field in {"suite", "assets", "output"})
    args = p.parse_args()
    from local_execution import register_auxiliary
    register_auxiliary(args,__file__)
    expected = {side: (lane["instance_id"], lane["gpu"]) for side, lane in PIPELINE_LANES.items()}
    if args.execution_json:
        from local_execution import validate
        resource=validate({'execution':json.loads(args.execution_json)})
        expected={side:(resource['instance_id'],resource['gpu_index']) for side in PIPELINE_LANES}
    if (args.instance, args.gpu) != expected[args.side] or os.environ.get("CUDA_VISIBLE_DEVICES") != str(args.gpu):
        raise ValueError("instance/GPU not authorized for this lane")
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        run(args)
    except Exception as e:
        if not (args.output / "failure.json").exists():
            write(args.output / "failure.json", {"error": str(e), "type": type(e).__name__})
        if args.barrier and args.barrier.is_dir() and not args.independent_replay:
            write(args.barrier / (args.side + "-failed.json"), {"error": str(e), "run_id": args.run_id})
        raise
