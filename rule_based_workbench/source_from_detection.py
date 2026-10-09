#!/usr/bin/env python3
"""Convert verified offline RGB-D detection output into the collection source contract.

This CPU-only bridge reads detection metadata, never scene object ground truth.
The adapter already transformed tracks.world into the Isaac environment frame.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from integration import (DualGpuPipeline, EARLY_CUP_MOTION_THRESHOLD_M, ObservationWindow,
                         PIPELINE_LANES, PIPELINE_SCHEMA, WINDOW_NS, _digest, _finite,
                         validate_profile)


def nonempty(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} is required")
    return value


def validate_physical_binding(binding, profile, summary, observations):
    if (not isinstance(binding, dict) or binding.get("calibration_accepted") is not True
            or binding.get("evidence_source") != "physical_rgbd"):
        raise ValueError("accepted physical calibration/acquisition binding required")
    nonempty(binding.get("robot_id"), "physical robot_id")
    serials = binding.get("camera_serials")
    if (not isinstance(serials, dict) or set(serials) != {"head", "left", "right"}
            or any(not isinstance(v, str) or not v.strip() for v in serials.values())
            or len(set(serials.values())) != 3):
        raise ValueError("three distinct physical camera serials required")
    if binding.get("calibration_profile_digest") != _digest(profile):
        raise ValueError("profile differs from accepted physical calibration")
    if (binding.get("detection_summary_digest") != _digest(summary)
            or binding.get("observations_digest") != _digest(observations)):
        raise ValueError("detection differs from trusted physical acquisition receipt")


def build_source(summary, observations, profile, run_id, physical_binding=None):
    validate_profile(profile)
    nonempty(run_id, "run_id")
    if physical_binding is not None:
        validate_physical_binding(physical_binding, profile, summary, observations)
    if (not isinstance(summary, dict) or summary.get("schema_version") != "isaac_rgbd_detection.v1"
            or summary.get("status") != "DETECTION_WINDOW_READY" or summary.get("cleanup_complete") is not True):
        raise ValueError("completed DETECTION_WINDOW_READY summary is required")
    if _finite(summary.get("valid_pair_span_s"), "valid_pair_span_s") < 5:
        raise ValueError("detection window must span 5 seconds")
    for key in ("adapter_sha256", "manifest_sha256", "node_config_sha256", "release_manifest_digest",
                "runtime_pack_digest", "runtime_config_id"):
        nonempty(summary.get(key), key)
    if not isinstance(summary.get("source"), dict) or not summary["source"]:
        raise ValueError("detection source revision evidence is required")
    inputs = summary.get("inputs")
    if not isinstance(inputs, list) or not isinstance(observations, list) or len(inputs) != len(observations):
        raise ValueError("every captured input must have one observation")
    calibration_thresholds = profile["divergence_thresholds"]
    summary_threshold = summary.get("cup_motion_threshold_m")
    profile_threshold = calibration_thresholds.get("cup_motion_threshold_m")
    explicit_threshold = summary_threshold if summary_threshold is not None else profile_threshold
    explicit_jitter = calibration_thresholds.get("cup_position_jitter_m")
    threshold_source = ("summary.cup_motion_threshold_m" if summary_threshold is not None
                        else "calibration_profile" if profile_threshold is not None
                        else "derived_from_initial_detection_jitter")
    if explicit_threshold is None:
        noise = summary.get("detection_noise")
        if isinstance(noise, dict):
            jitter = noise.get("cup_position_jitter_m")
            if isinstance(jitter, (int, float)) and not isinstance(jitter, bool):
                explicit_jitter = _finite(jitter, "detection_noise.cup_position_jitter_m")
                if explicit_jitter < 0:
                    raise ValueError("detection_noise.cup_position_jitter_m must be nonnegative")
                explicit_threshold = max(3.0 * explicit_jitter, EARLY_CUP_MOTION_THRESHOLD_M)
                threshold_source = "summary.detection_noise"
    if explicit_threshold is None and explicit_jitter is not None:
        explicit_jitter = _finite(explicit_jitter, "divergence_thresholds.cup_position_jitter_m")
        if explicit_jitter < 0:
            raise ValueError("divergence_thresholds.cup_position_jitter_m must be nonnegative")
        explicit_threshold = max(3.0 * explicit_jitter, EARLY_CUP_MOTION_THRESHOLD_M)
        threshold_source = "calibration_profile_jitter"
    window = ObservationWindow(profile, [], mode="offline_replay", coordinates="world",
                               cup_motion_threshold_m=explicit_threshold,
                               cup_position_jitter_m=explicit_jitter,
                               cup_motion_threshold_source=threshold_source)
    previous_stamp, previous_capture, previous_sequence = 0, -1, -1
    session, frame_id = None, None
    providers = []
    valid_stamps = []
    for capture, observation in zip(inputs, observations):
        if not isinstance(capture, dict) or not isinstance(observation, dict):
            raise ValueError("capture and observation must be objects")
        nonempty(capture.get("sha256"), "input sha256")
        if observation.get("mode") != "offline_replay":
            raise ValueError("offline detection must retain offline_replay mode")
        stamp, sequence, captured = (observation.get(k) for k in
                                    ("source_stamp_ns", "source_sequence_id", "capture_sequence_id"))
        if (any(type(x) is not int for x in (stamp, sequence, captured))
                or stamp <= previous_stamp or captured <= previous_capture or sequence <= previous_sequence
                or capture.get("source_stamp_ns") != stamp or capture.get("capture_sequence_id") != captured):
            raise ValueError("observation capture identity must match and strictly increase")
        previous_stamp, previous_capture, previous_sequence = stamp, captured, sequence
        observation_session = nonempty(observation.get("tracking_session_id"), "tracking_session_id")
        observation_frame = nonempty(observation.get("frame_id"), "frame_id")
        if session is None:
            session, frame_id = observation_session, observation_frame
        if (observation_session != session or observation_frame != frame_id
                or observation.get("runtime_config_id") != summary["runtime_config_id"]
                or observation.get("provider_status") not in {"OK", "SKIPPED"}):
            raise ValueError("observation runtime, frame or session changed, or provider failed")
        if type(observation.get("pair_valid")) is not bool:
            raise ValueError("pair_valid must be boolean")
        if not observation["pair_valid"]:
            window.frames.clear()
            providers.clear()
            continue  # Only the final contiguous run of valid pairs can define the scene.
        tracks = observation.get("tracks")
        if (not isinstance(tracks, list) or len(tracks) != 2
                or any(not isinstance(t, dict) or t.get("pose_current") is not True
                       or (t.get("object_type") == "coffee_machine" and t.get("orientation_ambiguous") is True)
                       for t in tracks)):
            raise ValueError("valid pair requires two current tracks and unambiguous coffee-machine orientation")
        valid_stamps.append(stamp)
        window_result = window.add(observation)
        if window_result.get("reobserve_required"):
            retained = {frame['source_stamp_ns'] for frame in window.frames}
            providers = [item for item in providers if item['source_stamp_ns'] in retained]
            if not retained:
                continue  # Immediate movement still discards its boundary sample.
        provenance = {}
        for track in observation["tracks"]:
            if track.get("object_type") not in {"coffee_machine", "cup"}:
                continue
            item = track.get("provenance")
            if not isinstance(item, dict):
                raise ValueError("track provider provenance is required")
            provider = nonempty(item.get("provider_id"), "provider_id")
            if provider != "foundationpose":
                raise ValueError("scene positions require the FoundationPose canonical mesh root; RGB-D surface centers need calibration")
            for key in ("provider_revision", "model_revision"):
                nonempty(item.get(key), key)
            if not isinstance(item.get("asset"), dict) or not item["asset"]:
                raise ValueError("object mesh identity is required")
            provenance[track["object_type"]] = deepcopy(item)
        providers.append({"source_stamp_ns": stamp, "capture_sequence_id": captured, "tracks": provenance})
        while len(window.frames) > 2 and stamp - window.frames[1]["source_stamp_ns"] >= WINDOW_NS:
            window.frames.pop(0)
            providers.pop(0)
    valid_span = (valid_stamps[-1] - valid_stamps[0]) / 1e9 if len(valid_stamps) >= 2 else 0
    if (type(summary.get("valid_pair_frames")) is not int or summary["valid_pair_frames"] != len(valid_stamps)
            or abs(summary["valid_pair_span_s"] - valid_span) > 1e-9):
        raise ValueError("summary valid window differs from observations")
    scene = window.finalize()
    scene["source"]["detection"] = {"summary": deepcopy(summary), "providers": providers,
                                      "observations_digest": _digest(observations),
                                      "reobserve_count": len(window.reobserve_events),
                                      "reobserve_events": deepcopy(window.reobserve_events),
                                      "cup_motion_threshold_m": window.cup_motion_threshold_m,
                                      "cup_position_jitter_m": window.cup_position_jitter_m,
                                      "cup_motion_threshold_source": window.cup_motion_threshold_source}
    scene["scene_digest"] = _digest({k: v for k, v in scene.items() if k != "scene_digest"})
    source = {"schema_version": PIPELINE_SCHEMA, **PIPELINE_LANES["simulated-real"],
              "lane": "simulated-real", "adapter": "foundationpose", "observation_mode": "offline_replay",
              "run_id": run_id, "sequence": 0, "scene": scene, "calibration_profile": deepcopy(profile)}
    if physical_binding is not None:
        source = {k: v for k, v in source.items() if k not in PIPELINE_LANES["simulated-real"]}
        source.update(schema_version="physical_collection_source.v1", lane="physical-real",
                      evidence_source="physical_rgbd", robot_id=physical_binding["robot_id"],
                      camera_serials=deepcopy(physical_binding["camera_serials"]))
        if 'initial_state' in physical_binding:
            from joint_space import validate_initial_state
            source['initial_state'] = deepcopy(validate_initial_state(physical_binding['initial_state'], profile))
    else:
        DualGpuPipeline().ingest_source(source)
    return source


def convert(profile_path, detection_dir, output, run_id, physical_binding=None, simulation_initial_suite=None):
    summary_bytes = (detection_dir / "summary.json").read_bytes()
    observation_bytes = (detection_dir / "observations.jsonl").read_bytes()
    summary = json.loads(summary_bytes)
    observations = [json.loads(line) for line in observation_bytes.splitlines() if line.strip()]
    detection_files = {"summary_sha256": hashlib.sha256(summary_bytes).hexdigest(),
                       "observations_sha256": hashlib.sha256(observation_bytes).hexdigest()}
    if physical_binding is not None:
        physical_binding = deepcopy(physical_binding)
        if physical_binding.get("detection_files") != detection_files:
            raise ValueError("detection files differ from trusted physical acquisition receipt")
        physical_binding.update(detection_summary_digest=_digest(summary),
                                observations_digest=_digest(observations))
    source = build_source(summary, observations, json.loads(profile_path.read_text()), run_id, physical_binding)
    source["detection_files"] = detection_files
    episode = deepcopy(source['scene']['episode_scene'])
    if simulation_initial_suite is not None:
        if physical_binding is not None:
            raise ValueError('simulation initial suite cannot bind a physical source')
        initial = json.loads(Path(simulation_initial_suite).read_text())['episodes']
        if len(initial)!=1:raise ValueError('one simulation initial episode required')
        metadata = initial[0].get('metadata') or {}
        model = metadata.get('initial_robot_state')
        from contract import validate_physical14
        validate_physical14(model,gripper_max_mm=source['calibration_profile']['gripper']['max_opening_mm'])
        if metadata.get('initial_robot_state_digest')!=_digest(model) or metadata.get('gripper_mode')!='distance':
            raise ValueError('sealed distance-mode simulation initial state required')
        episode['metadata'] = {key:deepcopy(metadata[key]) for key in ('initial_robot_state','initial_robot_state_digest','gripper_mode')}
    if 'initial_state' in source:
        model = source['initial_state']['model']
        episode['metadata'] = dict(initial_robot_state=deepcopy(model), initial_robot_state_digest=_digest(model))
    outputs = [("source.json", source), ("suite.json", {"episodes": [episode]})]
    if physical_binding is not None:
        physical_binding["source_digest"] = _digest(source)
        outputs.append(("hardware-binding.json", physical_binding))
    output.mkdir(parents=True, exist_ok=False)
    for filename, value in outputs:
        with (output / filename).open("x") as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("profile", "detection-dir", "output"):
        parser.add_argument("--" + flag, type=Path, required=True)
    parser.add_argument('--simulation-initial-suite',type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--physical-binding", type=Path, help="Trusted accepted physical acquisition/calibration receipt")
    args = parser.parse_args()
    if any(not path.is_absolute() for path in (args.profile, args.detection_dir, args.output)):
        parser.error("profile, detection-dir and output must be absolute paths")
    if args.physical_binding is not None and not args.physical_binding.is_absolute():
        parser.error("physical-binding must be an absolute path")
    if args.simulation_initial_suite is not None and not args.simulation_initial_suite.is_absolute():
        parser.error('simulation-initial-suite must be an absolute path')
    binding = json.loads(args.physical_binding.read_text()) if args.physical_binding else None
    source = convert(args.profile, args.detection_dir, args.output, args.run_id, binding,args.simulation_initial_suite)
    print(json.dumps({"source": str(args.output / "source.json"), "scene_digest": source["scene"]["scene_digest"],
                      "observation_mode": source["observation_mode"]}))


if __name__ == "__main__":
    main()
