"""Small, dependency-free bridge from ARX detection to the sim episode contract.

The bridge deliberately stops before Isaac and before hardware.  It owns only
the data semantics that must be identical on both sides of that boundary.
"""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

from contract import validate_physical14


PROFILE_SCHEMA = "calibration_profile.v1"
SCENE_SCHEMA = "scene_snapshot.v1"
RECEIPT_SCHEMA = "trajectory_receipt.v1"
WINDOW_NS = 3_000_000_000
# Numerical/statistical motion floor, independent of collision clearances.
# Unknown noise requires a full candidate window before motion classification.
EARLY_CUP_MOTION_THRESHOLD_M = 1e-3
SIM_PLANNER_ID = "coffee_episode_relative_rule_002"
CUROBO_PLANNER_ID = "curobo.MotionGen/0.7.8"
SIM_POLICY_ID = "coffee_relative_rule_development_004"
PIPELINE_SCHEMA = "dual_gpu_pipeline.v1"
PIPELINE_LANES = {
    "simulated-real": {"instance_id": "dsw-nygxkzfd5znx4if0t2", "gpu": 0},
    "dsw-scene": {"instance_id": "dsw-xs4hu2jkqj2igqaoul", "gpu": 0},
}


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _vec(value: Any, size: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must contain {size} values")
    return tuple(_finite(x, f"{name}[{i}]") for i, x in enumerate(value))


def _quat(value: Any, name: str) -> tuple[float, float, float, float]:
    q = _vec(value, 4, name)
    norm = math.sqrt(sum(x * x for x in q))
    if norm < 1e-9:
        raise ValueError(f"{name} must not be zero")
    q = tuple(x / norm for x in q)
    if next(x for x in q if abs(x) > 1e-12) < 0:
        q = tuple(-x for x in q)
    return q


def _digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def scene_errors(original, reconstructed):
    result = {}
    for name, key in (("machine", "environment_from_machine_root"), ("cup", "environment_from_cup")):
        a, b = original[key], reconstructed[key]
        result[name + "_position_error_m"] = math.sqrt(sum((x-y)**2 for x,y in zip(a["position"], b["position"])))
        if name == "machine":
            qa, qb = (_quat(pose["orientation_wxyz"], "machine orientation") for pose in (a, b))
            result["machine_rotation_error_rad"] = 2 * math.acos(min(1.0, abs(sum(x*y for x,y in zip(qa, qb)))))
    return result


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _qrotate(q, p):
    return _qmul(_qmul(q, (0.0, *p)), (q[0], -q[1], -q[2], -q[3]))[1:]


def _transform(transform, position, orientation):
    translation = _vec(transform["position"], 3, "environment_from_camera.position")
    rotation = _quat(transform["orientation_wxyz"], "environment_from_camera.orientation_wxyz")
    return tuple(a + b for a, b in zip(translation, _qrotate(rotation, position))), _qmul(rotation, orientation)


def _median_vectors(values):
    return tuple(statistics.median(row) for row in zip(*values))


def _mean_quaternion(values):
    reference = values[0]
    aligned = [q if sum(a * b for a, b in zip(q, reference)) >= 0 else tuple(-x for x in q) for q in values]
    return _quat(tuple(sum(q[i] for q in aligned) for i in range(4)), "machine orientation")


def validate_profile(profile: Any) -> dict[str, Any]:
    if not isinstance(profile, dict) or profile.get("schema_version") != PROFILE_SCHEMA:
        raise ValueError(f"schema_version must be {PROFILE_SCHEMA}")
    for key in ("profile_id", "source", "camera", "joints", "gripper", "frames", "divergence_thresholds"):
        if key not in profile:
            raise ValueError(f"calibration profile missing {key}")
    for key in ("source", "camera", "joints", "gripper", "frames"):
        if not isinstance(profile[key], dict):
            raise ValueError(f"calibration profile field {key} must be an object")
    if not isinstance(profile["profile_id"], str) or not profile["profile_id"].strip():
        raise ValueError("profile_id must be nonempty")
    camera = profile["camera"]
    _vec(camera.get("xyz_m"), 3, "camera.xyz_m")
    _vec(camera.get("rpy_rad"), 3, "camera.rpy_rad")
    _vec(profile["joints"].get("zero_offsets_rad"), 12, "joints.zero_offsets_rad")
    signs = _vec(profile["joints"].get("signs"), 12, "joints.signs")
    if any(sign not in (-1.0, 1.0) for sign in signs):
        raise ValueError("joint signs must be -1 or 1")
    if not isinstance(profile["gripper"].get("mapping"), str):
        raise ValueError("gripper.mapping is required")
    max_opening = _finite(profile["gripper"].get("max_opening_mm"), "gripper.max_opening_mm")
    if max_opening <= 0:
        raise ValueError("gripper.max_opening_mm must be positive")
    frames = profile["frames"]
    if not isinstance(frames.get("environment_from_camera"), dict):
        raise ValueError("frames.environment_from_camera is required")
    _vec(frames["environment_from_camera"].get("position"), 3, "environment_from_camera.position")
    _quat(frames["environment_from_camera"].get("orientation_wxyz"), "environment_from_camera.orientation_wxyz")
    thresholds = profile["divergence_thresholds"]
    if not isinstance(thresholds, dict) or not thresholds:
        raise ValueError("divergence_thresholds must be measured and nonempty")
    for key, value in thresholds.items():
        _finite(value, f"divergence_thresholds.{key}")
        if value < 0:
            raise ValueError(f"divergence_thresholds.{key} must be nonnegative")
    support = profile.get("scene_support")
    if support is not None:
        if not isinstance(support, dict):
            raise ValueError("scene_support requires a canonical machine asset")
        from scene_assets import asset_reference
        asset_reference(support.get('machine_asset'))
        if not isinstance(support.get("source"), str) or not support["source"].strip():
            raise ValueError("scene_support requires installation geometry provenance")
        for key in ("machine_root_z_m", "cup_root_z_m"):
            _finite(support.get(key), "scene_support." + key)
    return profile


@dataclass
class ObservationWindow:
    profile: dict[str, Any]
    frames: list[dict[str, Any]]
    mode: str = "live"
    coordinates: str = "camera"
    cup_motion_threshold_m: float | None = None
    cup_position_jitter_m: float | None = None
    cup_motion_threshold_source: str = "unresolved"
    reobserve_events: list[dict[str, Any]] = field(default_factory=list)
    asset_set: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.asset_set is not None:
            from scene_assets import validate
            self.asset_set = deepcopy(validate(self.asset_set))
        if self.mode not in {"live", "offline_replay"} or self.coordinates not in {"camera", "world"}:
            raise ValueError("unsupported observation mode or coordinate space")
        if self.cup_motion_threshold_m is not None:
            self.cup_motion_threshold_m = _finite(self.cup_motion_threshold_m, "cup_motion_threshold_m")
            if self.cup_motion_threshold_m <= 0:
                raise ValueError("cup_motion_threshold_m must be positive")
            if self.cup_motion_threshold_source == "unresolved":
                self.cup_motion_threshold_source = "provided_calibration"
        if self.cup_position_jitter_m is not None:
            self.cup_position_jitter_m = _finite(self.cup_position_jitter_m, "cup_position_jitter_m")
            if self.cup_position_jitter_m < 0:
                raise ValueError("cup_position_jitter_m must be nonnegative")

    def _learn_motion_threshold(self):
        # Unknown noise cannot be inferred from a single early displacement.
        # Assume approximately stationary, independent position noise; centered
        # step MAD avoids treating a persistent velocity as measurement jitter.
        if self.cup_motion_threshold_m is not None or len(self.frames) < 6:
            return None
        if self.frames[-1]["source_stamp_ns"] - self.frames[0]["source_stamp_ns"] < WINDOW_NS:
            return None
        points = [f["cup_position"] for f in self.frames]
        sigmas = []
        for axis in range(3):
            steps = [b[axis] - a[axis] for a, b in zip(points, points[1:])]
            center = statistics.median(steps)
            sigmas.append(1.4826 * statistics.median(abs(v-center) for v in steps) / math.sqrt(2))
        jitter = math.sqrt(sum(v*v for v in sigmas))
        # A sustained change of level (including monotonic drift) must not be
        # swallowed by a larger learned noise threshold. Require two samples
        # per side; a single outlier is not evidence of a persistent relocation.
        changes = []
        for split in range(2, len(points)-1):
            left = _median_vectors(points[:split]); right = _median_vectors(points[split:])
            distance = math.dist(left, right)
            limit = max(EARLY_CUP_MOTION_THRESHOLD_M,
                        4*jitter*math.sqrt(1/split + 1/(len(points)-split)))
            changes.append((distance/limit, distance, limit, split, left, right))
        ratio, distance, limit, split, left, right = max(changes)
        if ratio > 1:
            event = dict(source_stamp_ns=self.frames[-1]["source_stamp_ns"],
                         change_start_stamp_ns=self.frames[split]["source_stamp_ns"],
                         displacement_m=distance, threshold_m=limit,
                         reference_position_m=list(left), observed_position_m=list(right),
                         threshold_source="bootstrap_persistent_change")
            self.reobserve_events.append(event)
            # The bootstrap identified an earlier change boundary. Keep its
            # post-change samples; they still must pass a full fresh 3s test.
            del self.frames[:split]
            return event
        self.cup_position_jitter_m = jitter
        self.cup_motion_threshold_m = max(4*jitter, EARLY_CUP_MOTION_THRESHOLD_M)
        self.cup_motion_threshold_source = "derived_from_stationary_window"
        return None

    def add(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(snapshot, dict) or snapshot.get("mode") != self.mode:
            raise ValueError(f"only {self.mode}, non-stale detection snapshots can be collected")
        stamp = snapshot.get("source_stamp_ns")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise ValueError("source_stamp_ns must be a positive integer")
        if any(frame["source_stamp_ns"] == stamp for frame in self.frames):
            return self.status()
        frame_id = snapshot.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id is required")
        if self.frames and frame_id != self.frames[0]["frame_id"]:
            raise ValueError("frame_id changed inside the observation window")
        tracks = snapshot.get("tracks")
        if not isinstance(tracks, list) or any(not isinstance(t, dict) for t in tracks):
            raise ValueError("tracks must be a list of objects")
        current = [t for t in tracks if t.get("pose_current") is True]
        machines = [t for t in current if t.get("object_type") == "coffee_machine"]
        cups = [t for t in current if t.get("object_type") == "cup"]
        if len(machines) != 1 or len(cups) != 1:
            raise ValueError("a frame must contain current coffee_machine and cup tracks")
        machine, cup = machines[0], cups[0]
        if self.coordinates == "world":
            machine, cup = machine.get("world"), cup.get("world")
            if not isinstance(machine, dict) or not isinstance(cup, dict):
                raise ValueError("world poses are required for world observations")
        machine_position = _vec(machine.get("xyz_m"), 3, "coffee_machine.xyz_m")
        machine_q = _vec(machine.get("quaternion_xyzw"), 4, "coffee_machine.quaternion_xyzw")
        _quat(machine_q, "coffee_machine.quaternion_xyzw")
        cup_position = _vec(cup.get("xyz_m"), 3, "cup.xyz_m")
        if self.frames and self.cup_motion_threshold_m is not None:
            reference = _median_vectors([frame["cup_position"] for frame in self.frames])
            displacement = math.sqrt(sum((a - b) ** 2 for a, b in zip(cup_position, reference)))
            threshold = self.cup_motion_threshold_m or EARLY_CUP_MOTION_THRESHOLD_M
            if displacement > threshold:
                event = {
                    "source_stamp_ns": stamp,
                    "displacement_m": displacement,
                    "threshold_m": threshold,
                    "reference_position_m": list(reference),
                    "observed_position_m": list(cup_position),
                    "threshold_source": (self.cup_motion_threshold_source
                                          if self.cup_motion_threshold_m is not None
                                          else "early_action_gate"),
                }
                self.reobserve_events.append(event)
                self.frames.clear()
                return {**self.status(), "reobserve_required": True, "reobserve_event": event}
        self.frames.append(dict(source_stamp_ns=stamp, frame_id=frame_id, machine_position=machine_position,
                                machine_quaternion_xyzw=machine_q, cup_position=cup_position))
        self.frames.sort(key=lambda frame: frame["source_stamp_ns"])
        event = self._learn_motion_threshold()
        if event is not None:
            return {**self.status(), "reobserve_required": True, "reobserve_event": event}
        return self.status()

    def status(self):
        span = 0 if len(self.frames) < 2 else self.frames[-1]["source_stamp_ns"] - self.frames[0]["source_stamp_ns"]
        result = {"frame_count": len(self.frames), "span_s": span / 1e9, "ready": span >= WINDOW_NS and self.cup_motion_threshold_m is not None}
        if self.cup_motion_threshold_m is not None or self.reobserve_events:
            result.update(reobserve_count=len(self.reobserve_events),
                          cup_position_jitter_m=self.cup_position_jitter_m,
                          cup_motion_threshold_m=self.cup_motion_threshold_m,
                          cup_motion_threshold_source=self.cup_motion_threshold_source)
        return result

    def finalize(self) -> dict[str, Any]:
        if not self.status()["ready"]:
            raise ValueError("3 second stationary observation window is incomplete")
        frame = self.frames[0]
        transform = self.profile["frames"]["environment_from_camera"]
        machine_positions, machine_quaternions, cup_positions = [], [], []
        for sample in self.frames:
            machine_q = _quat((sample["machine_quaternion_xyzw"][3], sample["machine_quaternion_xyzw"][0],
                               sample["machine_quaternion_xyzw"][1], sample["machine_quaternion_xyzw"][2]), "machine quaternion")
            position, orientation = ((_transform(transform, sample["machine_position"], machine_q))
                                     if self.coordinates == "camera" else (sample["machine_position"], machine_q))
            machine_positions.append(position)
            machine_quaternions.append(orientation)
            cup_position = (_transform(transform, sample["cup_position"], (1.0, 0.0, 0.0, 0.0))[0]
                            if self.coordinates == "camera" else sample["cup_position"])
            cup_positions.append(cup_position)
        machine_position = _median_vectors(machine_positions)
        cup_position = _median_vectors(cup_positions)
        machine_orientation = _mean_quaternion(machine_quaternions)
        unconstrained = {"machine_position": list(machine_position), "machine_orientation_wxyz": list(machine_orientation),
                         "cup_position": list(cup_position)}
        support = self.profile.get("scene_support")
        from scene_assets import LEGACY_MACHINE
        machine_asset = self.asset_set['machine']['native_asset'] if self.asset_set is not None else LEGACY_MACHINE
        if support is not None and support['machine_asset'] != machine_asset:
            raise ValueError('installation support belongs to another machine asset; prepare the selected geometry first')
        if support is not None:
            # These canonical mesh roots are upright on a known installation plane.
            # Detection contributes XY/yaw; its roll/pitch must not tilt the tray.
            w, x, y, z = machine_orientation
            heading = (1 - 2 * (y*y + z*z), 2 * (w*z + x*y))
            if math.hypot(*heading) < 1e-6:
                raise ValueError("machine heading is undefined on the installation plane")
            yaw = math.atan2(heading[1], heading[0])
            machine_orientation = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
            machine_position = (*machine_position[:2], support["machine_root_z_m"])
            cup_position = (*cup_position[:2], support["cup_root_z_m"])
        scene = {
            "schema_version": SCENE_SCHEMA,
            "episode_id": "arx-coffee-observation",
            "seed": 0,
            "machine_asset": machine_asset,
            "environment_from_machine_root": {"position": list(machine_position), "orientation_wxyz": list(machine_orientation)},
            "environment_from_cup": {"position": list(cup_position), "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
            "source": {"frame_id": frame["frame_id"], "observation_mode": self.mode,
                       "input_coordinates": self.coordinates, "window_start_ns": self.frames[0]["source_stamp_ns"],
                       "window_end_ns": self.frames[-1]["source_stamp_ns"], "sample_count": len(self.frames),
                       "machine_orientation": "FoundationPose mesh canonical pose; averaged over stationary window",
                       "cup_orientation": "fixed upright; detection orientation ignored",
                       "reobserve_count": len(self.reobserve_events),
                       "reobserve_events": deepcopy(self.reobserve_events),
                       "cup_motion_threshold_m": self.cup_motion_threshold_m,
                       "cup_position_jitter_m": self.cup_position_jitter_m,
                       "cup_motion_threshold_source": self.cup_motion_threshold_source},
        }
        if support is not None:
            scene["source"].update(scene_support=deepcopy(support), unconstrained_detection=unconstrained,
                                   machine_orientation="detected yaw; upright and Z from installation geometry")
        scene["episode_scene"] = {key: scene[key] for key in (
            "episode_id", "seed", "machine_asset", "environment_from_machine_root", "environment_from_cup")}
        if self.asset_set is not None:
            scene['asset_set'] = deepcopy(self.asset_set)
        scene["scene_digest"] = _digest(scene)
        return scene


def _validate_timeline(ticks: Any, gripper_max_mm: float | None = None) -> None:
    if not isinstance(ticks, list) or not ticks:
        raise ValueError("trajectory ticks must be a nonempty list")
    previous_tick, previous_time = None, -1.0
    for item in ticks:
        if not isinstance(item, dict) or type(item.get("tick")) is not int or item["tick"] < 0:
            raise ValueError("trajectory tick must be a nonnegative integer")
        if previous_tick is not None and item["tick"] != previous_tick + 1:
            raise ValueError("trajectory ticks must be contiguous")
        time_s = _finite(item.get("time_s"), "tick.time_s")
        if time_s < 0 or time_s <= previous_time:
            raise ValueError("trajectory time_s must be nonnegative and strictly increasing")
        validate_physical14(item.get("target"), kind="trajectory target", gripper_max_mm=gripper_max_mm)
        previous_tick, previous_time = item["tick"], time_s


def freeze_receipt(payload: Any, profile: dict[str, Any], scene: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("trajectory payload must be an object")
    ticks = payload.get("ticks")
    _validate_timeline(ticks, profile["gripper"]["max_opening_mm"])
    planner_id, policy_id = payload.get("planner_id", SIM_PLANNER_ID), payload.get("policy_id", SIM_POLICY_ID)
    if planner_id != SIM_PLANNER_ID or policy_id != SIM_POLICY_ID:
        raise ValueError("trajectory must come from the current EpisodePlanner + RuleCoffeePolicy")
    simulation = payload.get("simulation")
    if not isinstance(simulation, dict):
        raise ValueError("simulation validation evidence is required")
    receipt = {"schema_version": RECEIPT_SCHEMA, "state": "sim_validated" if simulation.get("validated") is True else "awaiting_sim_validation",
               "scene_digest": scene.get("scene_digest"), "calibration_profile_id": profile["profile_id"],
               "calibration_profile_digest": _digest(profile), "planner_id": planner_id,
               "policy_id": policy_id, "source_revision": payload.get("source_revision"),
               "timeline_digest": _digest(ticks), "divergence_thresholds": profile["divergence_thresholds"], "ticks": ticks,
               "simulation": simulation}
    receipt["receipt_digest"] = _digest(receipt)
    return receipt


def _pipeline_lane(payload: Any, side: str) -> None:
    if not isinstance(payload, dict) or payload.get("schema_version") != PIPELINE_SCHEMA:
        raise ValueError(f"schema_version must be {PIPELINE_SCHEMA}")
    expected = PIPELINE_LANES[side]
    if (payload.get("instance_id") != expected["instance_id"] or
            type(payload.get("gpu")) is not int or payload["gpu"] != expected["gpu"]):
        raise ValueError(f"{side} requires {expected['instance_id']} GPU {expected['gpu']}")


def validate_assisted_scene(scene):
    """One accepted physical RGB-D estimate; never an automatic stable window."""
    import re
    observation=scene['source'];receipt=observation.get('assistance_receipt')
    if (not isinstance(receipt,dict) or receipt.get('schema_version')!='operator_assisted_rgbd_scene.v1'
            or receipt.get('status')!='operator_accepted' or receipt.get('scope')!='single_observation'
            or observation.get('automatic_window_qualified') is not False
            or observation.get('assistance_receipt_digest')!=_digest(receipt)):
        raise ValueError('explicit single-observation operator assistance receipt required')
    for key in ('request_id','robot_id','machine_asset'):
        if not isinstance(receipt.get(key),str) or not receipt[key].strip():raise ValueError('assistance '+key+' required')
    def sha(value):return isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value)
    if any(not sha(receipt.get(key)) for key in ('calibration_profile_digest','initial_state_digest')):
        raise ValueError('assistance requires calibration and measured-start digests')
    cameras=receipt.get('camera_serials')
    if (not isinstance(cameras,dict) or set(cameras)!={'head','left','right'}
            or any(not isinstance(v,str) or not v for v in cameras.values()) or len(set(cameras.values()))!=3):
        raise ValueError('assistance requires three camera identities')
    frames=receipt.get('frames')
    if not isinstance(frames,list) or not 1<=len(frames)<=256:raise ValueError('bounded actual RGB-D evidence required')
    paths=set();stamps=[]
    for frame in frames:
        if (not isinstance(frame,dict) or frame.get('camera') not in cameras
                or frame.get('camera_serial')!=cameras[frame['camera']]
                or not isinstance(frame.get('path'),str) or not frame['path'].startswith('/')
                or frame['path'] in paths or not sha(frame.get('sha256'))
                or type(frame.get('source_stamp_ns')) is not int or frame['source_stamp_ns']<=0
                or not isinstance(frame.get('frame_id'),str) or not frame['frame_id']):
            raise ValueError('assistance RGB-D frame identity/hash is invalid')
        paths.add(frame['path']);stamps.append(frame['source_stamp_ns'])
    if (type(receipt.get('accepted_wall_ns')) is not int or receipt['accepted_wall_ns']<max(stamps)
            or observation.get('window_start_ns')!=min(stamps) or observation.get('window_end_ns')!=max(stamps)
            or observation.get('sample_count')!=len(frames)):
        raise ValueError('assistance must retain actual evidence timestamps/counts')
    estimation=receipt.get('estimation')
    if (not isinstance(estimation,dict) or any(not isinstance(estimation.get(k),str) or not estimation[k].strip()
            for k in ('method','provenance_summary'))):
        raise ValueError('assistance estimation method and provenance required')
    for key in ('machine_asset','environment_from_machine_root','environment_from_cup'):
        if receipt.get(key)!=scene.get(key):raise ValueError('scene differs from accepted assistance geometry')
    geometry=receipt.get('cup_geometry')
    if (not isinstance(geometry,dict) or geometry.get('origin')!='bottom_center' or geometry.get('units')!='m'
            or not sha(geometry.get('profile_sha256'))):
        raise ValueError('assisted cup requires bottom-center metric geometry and actual profile digest')
    dimensions=_vec(geometry.get('dimensions_m'),3,'assisted cup dimensions')
    if any(value<=0 for value in dimensions):raise ValueError('positive assisted cup dimensions required')
    bands=geometry.get('bands')
    if not isinstance(bands,list) or not 1<=len(bands)<=32:raise ValueError('assisted cup collision bands required')
    previous=0.;radius=0.;names=set()
    for band in bands:
        if not isinstance(band,dict):raise ValueError('cup band must be an object')
        lo,hi,r=(_finite(band.get(k),'assisted cup '+k) for k in ('z_min_m','z_max_m','radius_m'))
        ends=[_finite(band[k] if band.get(k) is not None else r,'assisted cup '+k)
              for k in ('radius_start_m','radius_end_m')]
        name=band.get('shape_path')
        if (lo<0 or hi<=lo or r<=0 or abs(lo-previous)>1e-9 or min(ends)<=0 or max(ends)>r+1e-9
                or not isinstance(name,str) or not name.startswith('/Cup/Collisions/') or name in names):
            raise ValueError('finite contiguous assisted cup collision geometry required')
        previous=hi;radius=max(radius,r);names.add(name)
    if any(abs(a-b)>1e-9 for a,b in zip(dimensions,(2*radius,2*radius,previous))):
        raise ValueError('assisted cup dimensions differ from actual collision bands')
    grasp=receipt.get('grasp_command')
    if (not isinstance(grasp,dict) or grasp.get('units')!='metal_inner_gap_mm'
            or grasp.get('controlled_gripper')!='left_gripper'
            or type(grasp.get('metal_gap_mm')) not in (int,float)
            or grasp['metal_gap_mm'] not in (61.,65.)):
        raise ValueError('assisted observation requires the explicitly requested 61/65 mm left metal gap')
    return receipt


def _validate_scene(scene: Any, *, allow_operator_assisted=False) -> None:
    if not isinstance(scene, dict) or scene.get("schema_version") != SCENE_SCHEMA:
        raise ValueError(f"source scene must use {SCENE_SCHEMA}")
    if scene.get("scene_digest") != _digest({k: v for k, v in scene.items() if k != "scene_digest"}):
        raise ValueError("source scene digest does not match content")
    keys = ("episode_id", "seed", "machine_asset", "environment_from_machine_root", "environment_from_cup")
    expected = {key: scene[key] for key in keys if key in scene}
    if scene.get('asset_set') is not None:
        from scene_assets import validate
        validate(scene['asset_set'])
        if scene['machine_asset'] != scene['asset_set']['machine']['native_asset']:
            raise ValueError('observed scene machine differs from selected asset set')
    if any(key not in scene for key in keys) or scene.get("episode_scene") != expected:
        raise ValueError("episode_scene must match the observed scene")
    for key in keys[-2:]:
        pose = scene[key]
        if not isinstance(pose, dict):
            raise ValueError(f"{key} must be a pose")
        _vec(pose.get("position"), 3, f"{key}.position")
        _quat(pose.get("orientation_wxyz"), f"{key}.orientation_wxyz")
    observation = scene.get("source")
    if not isinstance(observation, dict):
        raise ValueError("scene observation window is required")
    if observation.get('observation_mode')=='operator_assisted_rgbd':
        if not allow_operator_assisted:raise ValueError('operator-assisted scene requires an explicit physical source binding')
        validate_assisted_scene(scene)
        return
    for key in ("window_start_ns", "window_end_ns", "sample_count"):
        if type(observation.get(key)) is not int or observation[key] <= 0:
            raise ValueError(f"scene source {key} must be positive integer")
    if observation["sample_count"] < 2 or observation["window_end_ns"] - observation["window_start_ns"] < WINDOW_NS:
        raise ValueError("scene requires a complete 3 second observation window")


def validate_sim_distance_plan(plan, source):
    """Simulation model-space distance commands; no physical calibration binding."""
    from gripper_distance_plan import HOLDING_STAGES, NATIVE_DISTANCE_ENCODING, holding_ticks, validate_release_control
    policy = plan.get('gripper_distance_control', {})
    if not isinstance(policy, dict):
        raise ValueError('simulation distance policy must be an object')
    profile = source['calibration_profile']
    offset = policy.get('native_travel_offset_mm')
    opening = plan.get('gripper_opening_mm')
    if (source.get('schema_version') != PIPELINE_SCHEMA or 'joint_space' in plan or 'initial_state' in source
            or profile['gripper'].get('mapping') != 'sum_of_two_prismatic_displacements_mm'
            or plan.get('native_action_encoding') != NATIVE_DISTANCE_ENCODING
            or policy.get('schema_version') != 'sim_gripper_distance_control.v1'
            or policy.get('closed_metal_gap_mm') != 65.0 or policy.get('rubber_gap_mm') != 62.0
            or policy.get('pad_thickness_m') != .0015 or policy.get('holding_stages') != sorted(HOLDING_STAGES)
            or type(offset) not in (int, float) or not math.isfinite(offset)
            or type(opening) not in (int, float) or not math.isfinite(opening)
            or not 0 < opening <= profile['gripper']['max_opening_mm']
            or not 0 <= 65.0-offset <= opening):
        raise ValueError('explicit simulation distance encoding and 65/62 mm metal/rubber policy required')
    held = holding_ticks(plan)
    actions, counts = plan.get('native_actions'), plan.get('native_substeps')
    control = plan.get('control', {})
    hz, nominal = control.get('physics_hz'), control.get('substeps')
    duration = plan.get('duration_s')
    if (not any(held) or not isinstance(actions, list) or not isinstance(counts, list)
            or len(actions) != len(plan['ticks']) or len(counts) != len(actions)
            or type(hz) is not int or type(nominal) is not int or hz <= 0 or nominal <= 0 or hz != 30*nominal
            or any(type(n) is not int or not 0 < n <= nominal for n in counts)
            or any(n != nominal for n in counts[:-1])
            or type(duration) not in (int, float) or not math.isfinite(duration)
            or not math.isclose(duration, sum(counts)/hz, rel_tol=0, abs_tol=1e-9)):
        raise ValueError('distance actions require complete 30 Hz native cadence and holding stages')
    for tick, action, holding in zip(plan['ticks'], actions, held):
        if (not isinstance(action, list) or len(action) != 14
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in action)
                or any(not -1 <= v <= 1 for slot, v in enumerate(action) if slot not in (6, 13))):
            raise ValueError('distance actions retain twelve normalized arm coordinates')
        for side, slot in (('left', 6), ('right', 13)):
            if not 0 <= action[slot] <= opening or abs(action[slot]-tick['target'][side+'_gripper']) > 1e-9:
                raise ValueError('simulation gripper target differs from explicit native travel mm')
        if holding and abs(tick['target']['left_gripper']-(65.0-offset)) > 1e-9:
            raise ValueError('holding target differs from model-adjusted 65 mm metal gap')
    closing=holding_ticks(plan,{'close_ramp'})
    if any(closing):
        curve=plan.get('gripper_closing',{})
        rows=[tick['target'] for tick,active in zip(plan['ticks'],closing) if active]
        if (curve.get('schema_version')!='commanded_gap_closing.v1' or curve.get('maximum_step_mm')!=2.
                or abs(rows[0]['left_gripper']-opening)>1e-9 or abs(rows[-1]['left_gripper']-(65.-offset))>1e-9
                or any(any(row[k]!=rows[0][k] for k in ('left_arm_joints','right_arm_joints','right_gripper')) for row in rows)
                or any(not 0<=a['left_gripper']-b['left_gripper']<=2.00000001 for a,b in zip(rows,rows[1:]))):
            raise ValueError('closing requires stationary arms and a continuous open-to-command ramp')
    validate_release_control(plan, closed=65.0-offset, opening=opening)
    return held


@dataclass
class DualGpuPipeline:
    """Validate the two authorized DSW lanes; replay alone is not wall-clock sync."""

    source: dict[str, Any] | None = None
    plan: dict[str, Any] | None = None
    executions: dict[str, dict[str, Any]] | None = None

    def __post_init__(self) -> None:
        if self.executions is None:
            self.executions = {}

    def ingest_source(self, payload: Any) -> dict[str, Any]:
        _pipeline_lane(payload, "simulated-real")
        if payload.get("lane") != "simulated-real":
            raise ValueError("source lane must be simulated-real")
        if payload.get("adapter") not in {"sim-online-detection-ground-truth", "foundationpose"}:
            raise ValueError("unsupported online detection adapter")
        _validate_scene(payload.get("scene"))
        validate_profile(payload.get("calibration_profile"))
        if not isinstance(payload.get("run_id"), str) or not payload["run_id"].strip():
            raise ValueError("source run_id is required")
        sequence = payload.get("sequence")
        if type(sequence) is not int or sequence < 0:
            raise ValueError("source sequence must be a nonnegative integer")
        if self.source and payload["run_id"] == self.source["run_id"] and sequence <= self.source["sequence"]:
            if payload == self.source:
                return self.summary()
            raise ValueError("source sequence must advance within a run")
        self.source = deepcopy(payload)
        self.plan = None
        self.executions = {}
        return self.summary()

    def _source_reference(self, payload: dict[str, Any]) -> None:
        if not self.source:
            raise ValueError("source observation must arrive first")
        expected = {"source_run_id": self.source["run_id"], "source_sequence": self.source["sequence"],
                    "scene_digest": self.source["scene"]["scene_digest"],
                    "calibration_profile_digest": _digest(self.source["calibration_profile"])}
        if type(payload.get("source_sequence")) is not int or any(payload.get(k) != v for k, v in expected.items()):
            raise ValueError("source run, sequence, scene or calibration profile differs")

    def accept_plan(self, payload: Any) -> dict[str, Any]:
        _pipeline_lane(payload, "dsw-scene")
        if payload.get("lane") != "dsw-scene":
            raise ValueError("target lane must be dsw-scene")
        self._source_reference(payload)
        if not isinstance(payload.get("run_id"), str) or not payload["run_id"].strip():
            raise ValueError("plan run_id is required")
        if payload.get("planner_id") not in (SIM_PLANNER_ID, CUROBO_PLANNER_ID) or payload.get("policy_id") != SIM_POLICY_ID:
            raise ValueError("target plan identity differs from the sim planner/policy")
        _validate_timeline(payload.get("ticks"), self.source["calibration_profile"]["gripper"]["max_opening_mm"])
        if payload.get("timeline_digest") != _digest(payload["ticks"]):
            raise ValueError("plan timeline digest does not match ticks")
        if payload.get('planner_id') == CUROBO_PLANNER_ID:
            backend = payload.get('planning_backend', {})
            receipt = payload.get('planning_validation', {})
            if (not isinstance(backend, dict) or not isinstance(receipt, dict) or payload.get('offline_candidate') is True
                    or backend.get('backend') != 'curobo_motiongen' or backend.get('curobo_version') != '0.7.8'
                    or backend.get('gpu_ik') is not True or backend.get('gpu_trajectory_search') is not True
                    or receipt.get('schema_version') != 'curobo_motiongen_path_validation.v1'
                    or receipt.get('passed') is not True or receipt.get('offline_candidate') is True
                    or receipt.get('source_digest') != _digest(self.source)
                    or receipt.get('timeline_digest') != payload['timeline_digest']
                    or payload.get('simulation') != {'validated': False, 'outcome': 'not_run'}):
                raise ValueError('MotionGen requires bound GPU search evidence; task success awaits dual replay')
            validate_sim_distance_plan(payload, self.source)
            from physical_motiongen import validate_grasp_contact_evidence
            validate_grasp_contact_evidence(payload,self.source)
        if self.plan == payload:
            return self.summary()
        self.plan = deepcopy(payload)
        self.executions = {}
        return self.summary()

    def record_execution(self, payload: Any) -> dict[str, Any]:
        if not isinstance(payload, dict) or not isinstance(payload.get("side"), str) or payload["side"] not in PIPELINE_LANES:
            raise ValueError("execution side must be simulated-real or dsw-scene")
        side = payload["side"]
        _pipeline_lane(payload, side)
        self._source_reference(payload)
        if not self.plan:
            raise ValueError("target plan must be accepted before execution")
        if payload.get("plan_run_id") != self.plan["run_id"] or payload.get("timeline_digest") != self.plan["timeline_digest"]:
            raise ValueError("execution differs from frozen target plan")
        if payload.get("status") != "completed" or type(payload.get("tick_count")) is not int or payload["tick_count"] != len(self.plan["ticks"]):
            raise ValueError("execution must complete every planned tick")
        _validate_timeline(payload.get("applied_ticks"), self.source["calibration_profile"]["gripper"]["max_opening_mm"])
        if _digest(payload["applied_ticks"]) != self.plan["timeline_digest"]:
            raise ValueError("applied ticks differ from frozen timeline")
        measured = payload.get("measured_states")
        if not isinstance(measured, list) or len(measured) != payload["tick_count"]:
            raise ValueError("execution requires measured physical14 state for every tick")
        for state in measured:
            validate_physical14(state, kind="measured state")
        actual = payload.get("actual_episode_scene")
        if actual is not None:
            expected = self.source["scene"]["episode_scene"]
            if (not isinstance(actual, dict) or set(actual) != set(expected)
                    or payload.get("actual_episode_digest") != _digest(actual)
                    or payload.get("detected_scene_digest") != self.source["scene"]["scene_digest"]):
                raise ValueError("actual execution scene identity is incomplete or inconsistent")
            for key in ("environment_from_machine_root", "environment_from_cup"):
                if not isinstance(actual.get(key), dict):
                    raise ValueError("actual execution scene pose is required")
                _vec(actual[key].get("position"), 3, "actual position")
                _quat(actual[key].get("orientation_wxyz"), "actual orientation")
            if side == "dsw-scene" and actual != expected:
                raise ValueError("planning lane must execute the detected scene")
        measured_scene = payload.get("measured_reset_scene")
        if measured_scene is not None:
            if (not isinstance(measured_scene, dict)
                    or set(measured_scene) != {"environment_from_machine_root", "environment_from_cup"}
                    or payload.get("measured_scene_source") != "isaac_post_reset_mesh_roots"):
                raise ValueError("measured scene requires explicit Isaac post-reset mesh-root provenance")
            for pose in measured_scene.values():
                if not isinstance(pose, dict):
                    raise ValueError("measured scene pose must be an object")
                _vec(pose.get("position"), 3, "measured scene position")
                _quat(pose.get("orientation_wxyz"), "measured scene orientation")
        timing = payload.get("timing")
        if timing is not None:
            if not isinstance(timing, dict) or timing.get("clock") != "utc":
                raise ValueError("execution timing must use a measured UTC clock")
            uncertainty = _finite(timing.get("uncertainty_s"), "clock uncertainty_s")
            if uncertainty < 0 or not isinstance(timing.get("evidence"), str) or not timing["evidence"].strip():
                raise ValueError("clock uncertainty must be nonnegative and have evidence")
            stamps = _vec(timing.get("applied_at_unix_s"), payload["tick_count"], "applied_at_unix_s")
            if stamps[0] <= 0 or any(b <= a for a, b in zip(stamps, stamps[1:])):
                raise ValueError("applied UTC timestamps must be positive and strictly increasing")
        self.executions[side] = deepcopy(payload)
        return self.summary()

    def summary(self) -> dict[str, Any]:
        source, plan, executions = self.source, self.plan, self.executions or {}
        aligned = bool(source and plan)
        complete = aligned and PIPELINE_LANES.keys() <= executions.keys()
        execution = "replay-complete" if complete else ("in-progress" if executions else "waiting")
        timing_status, max_skew, start_skew, max_schedule_error = "unverified", None, None, None
        scene_comparison = {"status": "unverified"}
        if complete and all(executions[side].get("actual_episode_scene") for side in PIPELINE_LANES):
            original, reconstructed = (executions[side]["actual_episode_scene"] for side in PIPELINE_LANES)
            scene_comparison = {"status": "configured", "scope": "requested reset poses of the source scene versus reconstruction; not post-settle measured poses"}
            scene_comparison.update(scene_errors(original, reconstructed))
        measured_comparison = {"status": "unverified"}
        if complete and all(executions[side].get("measured_reset_scene") for side in PIPELINE_LANES):
            original, reconstructed = (executions[side]["measured_reset_scene"] for side in PIPELINE_LANES)
            measured_comparison = {"status": "measured", "scope": "Isaac post-reset mesh roots; audit only, not perception input",
                                   **scene_errors(original, reconstructed)}
        if complete and all(executions[side].get("timing") for side in PIPELINE_LANES):
            left, right = (executions[side]["timing"] for side in PIPELINE_LANES)
            a, b = left["applied_at_unix_s"], right["applied_at_unix_s"]
            uncertainty = left["uncertainty_s"] + right["uncertainty_s"]
            start_skew = abs(a[0] - b[0])
            max_skew = max(abs(x - y) for x, y in zip(a, b)) + uncertainty
            target_times = [tick["time_s"] - plan["ticks"][0]["time_s"] for tick in plan["ticks"]]
            max_schedule_error = max(abs(stamp - stamps[0] - t) for stamps in (a, b)
                                     for stamp, t in zip(stamps, target_times)) + uncertainty
            tolerance = source["calibration_profile"]["divergence_thresholds"].get("timing_s")
            if tolerance is not None:
                timing_status = "passed" if max(max_skew, max_schedule_error) <= tolerance else "exceeded"
                if timing_status == "passed":
                    execution = "synchronized"
        return {
            "schema_version": PIPELINE_SCHEMA,
            "source_lane": {**PIPELINE_LANES["simulated-real"], "status": "ready" if source else "awaiting-observation",
                            "adapter": source.get("adapter") if source else None,
                            "scene_digest": source["scene"]["scene_digest"] if source else None},
            "target_lane": {**PIPELINE_LANES["dsw-scene"], "status": "planned" if plan else "awaiting-scene",
                            "scene_digest": plan.get("scene_digest") if plan else None,
                            "planner_id": plan.get("planner_id") if plan else None},
            "alignment": "aligned" if aligned else "waiting", "execution": execution,
            "alignment_scope": "detected scene transferred to planner; physical pose accuracy is separate",
            "scene_comparison": scene_comparison,
            "measured_scene_comparison": measured_comparison,
            "recording": {side: executions[side].get("recording", {"capture_status": "unverified"}) for side in executions},
            "timing": {"status": timing_status, "start_skew_s": start_skew,
                       "max_skew_with_uncertainty_s": max_skew, "max_schedule_error_s": max_schedule_error},
            "timeline_digest": plan.get("timeline_digest") if plan else None,
            "tick_count": len(plan["ticks"]) if plan else 0, "execution_lanes": sorted(executions),
        }
