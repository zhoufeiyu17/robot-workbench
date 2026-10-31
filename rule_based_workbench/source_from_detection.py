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
from real_contract import namespace, validate_namespace

from integration import (DualGpuPipeline, EARLY_CUP_MOTION_THRESHOLD_M, ObservationWindow,
                         PIPELINE_LANES, PIPELINE_SCHEMA, WINDOW_NS, _digest, _finite,
                         validate_profile)


def nonempty(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} is required")
    return value


def validate_physical_binding(binding, profile, summary, observations, *, environment='physical'):
    validate_namespace(binding, environment)
    if (not isinstance(binding, dict) or binding.get("calibration_accepted") is not True
            or environment == 'fake' and profile['source'].get('simulation_only') is not True):
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


def build_source(summary, observations, profile, run_id, physical_binding=None, *, environment='physical', asset_set=None):
    validate_profile(profile)
    nonempty(run_id, "run_id")
    if physical_binding is not None:
        validate_physical_binding(physical_binding, profile, summary, observations, environment=environment)
    elif environment != 'physical':
        raise ValueError('fake-device source requires explicit accepted calibration/acquisition binding')
    if (not isinstance(summary, dict) or summary.get("schema_version") != "isaac_rgbd_detection.v1"
            or summary.get("status") != "DETECTION_WINDOW_READY" or summary.get("cleanup_complete") is not True):
        raise ValueError("completed DETECTION_WINDOW_READY summary is required")
    if _finite(summary.get("valid_pair_span_s"), "valid_pair_span_s") < 3:
        raise ValueError("detection window must span 3 seconds")
    for key in ("adapter_sha256", "manifest_sha256", "node_config_sha256", "release_manifest_digest",
                "runtime_pack_digest", "runtime_config_id"):
        nonempty(summary.get(key), key)
    if not isinstance(summary.get("source"), dict) or not summary["source"]:
        raise ValueError("detection source revision evidence is required")
    inputs = summary.get("inputs")
    if not isinstance(inputs, list) or not isinstance(observations, list) or len(inputs) != len(observations):
        raise ValueError("every captured input must have one observation")
    from robot_workbench.observation_window import ContinuousObservationWindow, CUP_SPAN_M
    continuous = ContinuousObservationWindow()
    window = ObservationWindow(profile, [], mode="offline_replay", coordinates="world",
                               cup_motion_threshold_m=CUP_SPAN_M,
                               cup_motion_threshold_source="approved_continuous_window_6mm")
    previous_stamp, previous_capture, previous_sequence = 0, -1, -1
    session, frame_id = None, None
    providers = []
    valid_stamps = []
    if asset_set is not None:
        from scene_assets import check_detection
        check_detection(asset_set, summary)
        window.asset_set = deepcopy(asset_set)
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
        continuous.add(observation)
        if not observation["pair_valid"]:
            continue
        tracks = observation.get("tracks")
        if (not isinstance(tracks, list) or len(tracks) != 2
                or any(not isinstance(t, dict) or t.get("pose_current") is not True
                       or (t.get("object_type") == "coffee_machine" and t.get("orientation_ambiguous") is True)
                       for t in tracks)):
            raise ValueError("valid pair requires two current tracks and unambiguous coffee-machine orientation")
        valid_stamps.append(stamp)
        if asset_set is not None:
            check_detection(asset_set, summary, tracks)
            from cup_pose_refinement import validate_observation
            validate_observation(observation,asset_set['cup'],input_sha256=capture['sha256'])
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
    valid_span = (valid_stamps[-1] - valid_stamps[0]) / 1e9 if len(valid_stamps) >= 2 else 0
    if (type(summary.get("valid_pair_frames")) is not int or summary["valid_pair_frames"] != len(valid_stamps)
            or abs(summary["valid_pair_span_s"] - valid_span) > 1e-9):
        raise ValueError("summary valid window differs from observations")
    status = continuous.status()
    if not status['ready']:
        raise ValueError('continuous stable detection requires nine valid frames spanning three seconds')
    if (summary.get('continuous_valid_pair_stamps_ns') is not None and
            summary['continuous_valid_pair_stamps_ns'] != status['valid_stamps_ns']):
        raise ValueError('summary terminal window differs from independently checked observations')
    for observation in continuous.frames:
        window.add(observation)
    window.reobserve_events = deepcopy(continuous.events)
    accepted_stamps = set(status['valid_stamps_ns'])
    providers = [item for item in providers if item['source_stamp_ns'] in accepted_stamps]
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
    if asset_set is not None and asset_set['cup'].get('grasp_command') is not None:
        source['grasp_command']=deepcopy(asset_set['cup']['grasp_command'])
    if physical_binding is not None:
        source = {k: v for k, v in source.items() if k not in PIPELINE_LANES["simulated-real"]}
        spec = namespace(environment)
        source.update(schema_version=spec['source_schema'], lane=spec['lane'],
                      evidence_source=spec['evidence'], robot_id=physical_binding["robot_id"],
                      camera_serials=deepcopy(physical_binding["camera_serials"]))
        if environment == 'fake':source.update(environment='fake', simulation_only=True)
        if environment == 'fake':
            from robot_workbench.sample_clock import clock_for_source
            source['clock']=clock_for_source(physical_binding,environment)
        if 'initial_state' in physical_binding:
            from joint_space import validate_initial_state
            source['initial_state'] = deepcopy(validate_initial_state(physical_binding['initial_state'], profile))
    else:
        DualGpuPipeline().ingest_source(source)
    return source


def convert(profile_path, detection_dir, output, run_id, physical_binding=None, simulation_initial_suite=None, *, environment='physical', assets=None, asset_set_digest=None):
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
    from scene_assets import load
    selection = load(assets, expected_digest=asset_set_digest) if assets is not None else None
    if asset_set_digest is not None and assets is None:
        raise ValueError('selected asset set requires its prepared native assets root')
    source = build_source(summary, observations, json.loads(profile_path.read_text()), run_id, physical_binding,
                          environment=environment, asset_set=selection)
    source["detection_files"] = detection_files
    episode = deepcopy(source['scene']['episode_scene'])
    if selection is not None:
        from scene_assets import episode_metadata
        episode['metadata'] = episode_metadata(selection)
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
        episode.setdefault('metadata', {}).update({key:deepcopy(metadata[key]) for key in ('initial_robot_state','initial_robot_state_digest','gripper_mode')})
    if 'initial_state' in source:
        model = source['initial_state']['model']
        episode.setdefault('metadata', {}).update(initial_robot_state=deepcopy(model), initial_robot_state_digest=_digest(model))
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


def build_assisted_source(receipt, profile, run_id, physical_binding):
    """Explicit one-off human-assisted source; no synthetic detection summaries."""
    from integration import SCENE_SCHEMA, _validate_scene
    from real_observation import validate_binding
    from real_contract import validate_real_source
    from joint_space import validate_initial_state
    validate_binding(profile,physical_binding,environment='physical')
    nonempty(run_id,'run_id')
    initial=deepcopy(validate_initial_state(physical_binding.get('initial_state'),profile))
    frames=receipt.get('frames') if isinstance(receipt,dict) else None
    if not isinstance(frames,list) or not frames:raise ValueError('actual assistance RGB-D frames required')
    stamps=[frame.get('source_stamp_ns') for frame in frames]
    if any(type(stamp) is not int for stamp in stamps):raise ValueError('actual assistance frame timestamps required')
    scene=dict(schema_version=SCENE_SCHEMA,episode_id=run_id,seed=0,
        **{key:deepcopy(receipt.get(key)) for key in ('machine_asset','environment_from_machine_root','environment_from_cup')},
        source=dict(frame_id='environment',observation_mode='operator_assisted_rgbd',input_coordinates='world',
            automatic_window_qualified=False,window_start_ns=min(stamps),window_end_ns=max(stamps),sample_count=len(frames),
            assistance_receipt=deepcopy(receipt),assistance_receipt_digest=_digest(receipt)))
    scene['episode_scene']={key:deepcopy(scene[key]) for key in
        ('episode_id','seed','machine_asset','environment_from_machine_root','environment_from_cup')}
    scene['scene_digest']=_digest(scene)
    _validate_scene(scene,allow_operator_assisted=True)
    geometry=deepcopy(receipt['cup_geometry'])
    source=dict(schema_version='physical_collection_source.v1',lane='physical-real',evidence_source='physical_rgbd',
        adapter='operator_assisted_rgbd',observation_mode='operator_assisted_rgbd',run_id=run_id,sequence=0,
        robot_id=physical_binding['robot_id'],camera_serials=deepcopy(physical_binding['camera_serials']),
        calibration_profile=deepcopy(profile),initial_state=initial,scene=scene,
        cup_geometry=geometry,cup_geometry_digest=_digest(geometry),grasp_command=deepcopy(receipt['grasp_command']))
    binding=deepcopy(physical_binding)
    binding.update(source_digest=_digest(source),operator_assistance_digest=_digest(receipt),
        operator_assistance_request_id=receipt['request_id'],cup_geometry_digest=_digest(geometry))
    validate_real_source(source,binding,environment='physical')
    episode=deepcopy(scene['episode_scene'])
    # Native episodes accept only their initial state pair. Assistance remains
    # fully bound in source.json and hardware-binding.json, not duplicate metadata.
    episode['metadata']=dict(initial_robot_state=deepcopy(initial['model']),initial_robot_state_digest=_digest(initial['model']))
    return source,dict(episodes=[episode]),binding


def convert_assisted(profile_path, receipt_path, output, run_id, physical_binding):
    """Write the existing three-artifact planner input; verify, never copy, RGB-D."""
    receipt=json.loads(Path(receipt_path).read_text())
    profile=json.loads(Path(profile_path).read_text())
    source,suite,binding=build_assisted_source(receipt,profile,run_id,physical_binding)
    for frame in receipt['frames']:
        path=Path(frame['path'])
        if (not path.is_file() or path.is_symlink() or path.suffix!='.npz'
                or hashlib.sha256(path.read_bytes()).hexdigest()!=frame['sha256']):
            raise ValueError('assisted raw RGB-D evidence file/hash differs')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    for name,value in (('source.json',source),('suite.json',suite),('hardware-binding.json',binding)):
        (output/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("profile", "detection-dir", "output"):
        parser.add_argument("--" + flag, type=Path, required=True)
    parser.add_argument('--simulation-initial-suite',type=Path)
    parser.add_argument('--assets', type=Path)
    parser.add_argument('--asset-set-digest')
    parser.add_argument('--environment', choices=('physical','fake'), default='physical')
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--physical-binding", type=Path, help="Trusted accepted physical acquisition/calibration receipt")
    args = parser.parse_args()
    if any(not path.is_absolute() for path in (args.profile, args.detection_dir, args.output)):
        parser.error("profile, detection-dir and output must be absolute paths")
    if args.physical_binding is not None and not args.physical_binding.is_absolute():
        parser.error("physical-binding must be an absolute path")
    if args.simulation_initial_suite is not None and not args.simulation_initial_suite.is_absolute():
        parser.error('simulation-initial-suite must be an absolute path')
    if args.assets is not None and not args.assets.is_absolute():
        parser.error('assets must be an absolute path')
    binding = json.loads(args.physical_binding.read_text()) if args.physical_binding else None
    source = convert(args.profile, args.detection_dir, args.output, args.run_id, binding,args.simulation_initial_suite,
                     environment=args.environment,assets=args.assets,asset_set_digest=args.asset_set_digest)
    print(json.dumps({"source": str(args.output / "source.json"), "scene_digest": source["scene"]["scene_digest"],
                      "observation_mode": source["observation_mode"]}))


if __name__ == "__main__":
    main()
