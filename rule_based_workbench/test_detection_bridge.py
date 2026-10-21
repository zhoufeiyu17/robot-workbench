"""CPU contract fixtures only; these checks are not FoundationPose inference evidence."""
from copy import deepcopy
import math
import json
from pathlib import Path
import tempfile

from integration import ObservationWindow
from source_from_detection import build_source, convert
from test_harness import PROFILE, rejects, snapshot


def fixture():
    observations = []
    for i in range(9):
        row = snapshot(1_000_000_000 + i * 375_000_000)
        row.update(mode="offline_replay", pair_valid=True, provider_status="OK", source_sequence_id=i,
                   capture_sequence_id=i, tracking_session_id="fixture", runtime_config_id="fixture")
        for number, track in enumerate(row["tracks"]):
            track["track_id"] = number
            track["world"] = {"xyz_m": [1., 2., 3.]}
            track["provenance"] = {"provider_id": "foundationpose", "provider_revision": "fixture",
                                    "model_revision": "fixture", "asset": {"asset_id": "fixture"}}
        row["tracks"][0]["world"]["quaternion_xyzw"] = [0., 0., 2 ** -.5, 2 ** -.5]
        row["tracks"][0]["provenance"] = {"provider_id": "foundationpose", "provider_revision": "fixture",
                                             "model_revision": "fixture", "asset": {"asset_id": "fixture"}}
        row["tracks"][1]["world"]["quaternion_xyzw"] = ["deliberately ignored"]
        observations.append(row)
    summary = {"schema_version": "isaac_rgbd_detection.v1", "status": "DETECTION_WINDOW_READY",
               "cleanup_complete": True, "valid_pair_frames": 9, "valid_pair_span_s": 3., "source": {"revision": "fixture"},
               "runtime_config_id": "fixture", "inputs": [{"source_stamp_ns": r["source_stamp_ns"],
               "capture_sequence_id": r["capture_sequence_id"], "sha256": "a" * 64} for r in observations]}
    for key in ("adapter_sha256", "manifest_sha256", "node_config_sha256", "release_manifest_digest", "runtime_pack_digest"):
        summary[key] = "b" * 64
    profile = deepcopy(PROFILE)
    profile["frames"]["environment_from_camera"]["position"] = [100., 200., 300.]
    return summary, observations, profile


def main():
    summary, observations, profile = fixture()
    source = build_source(summary, observations, profile, "fixture")
    scene = source["scene"]
    assert source["observation_mode"] == scene["source"]["observation_mode"] == "offline_replay"
    assert source["adapter"] == "foundationpose" and source["calibration_profile"] == profile
    assert scene["environment_from_machine_root"]["position"] == [1., 2., 3.]  # No second camera transform.
    assert all(abs(a-b)<1e-12 for a,b in zip(scene["environment_from_machine_root"]["orientation_wxyz"], [2 ** -.5, 0., 0., 2 ** -.5]))
    assert scene["environment_from_cup"] == {"position": [1., 2., 3.], "orientation_wxyz": [1., 0., 0., 0.]}
    assert scene["source"]["detection"]["summary"] == summary
    constrained_profile = deepcopy(profile)
    constrained_profile["scene_support"] = {"machine_asset": "objects/owner_coffee_machine_00003/v1",
        "machine_root_z_m": .78, "cup_root_z_m": .78, "source": "fixture installation plane"}
    tilted = deepcopy(observations)
    # Observed 90-degree yaw plus roll error; neither roll nor noisy Z may
    # change the known tray plane. Keep raw detections available for audit.
    for row in tilted:
        row["tracks"][0]["world"]["quaternion_xyzw"] = [math.sin(.02) / 2**.5,
            math.sin(.02) / 2**.5, math.cos(.02) / 2**.5, math.cos(.02) / 2**.5]
    corrected = build_source(summary, tilted, constrained_profile, "fixture")["scene"]
    assert corrected["environment_from_machine_root"]["position"] == [1., 2., .78]
    assert corrected["environment_from_cup"]["position"] == [1., 2., .78]
    assert all(abs(a-b) < 1e-12 for a,b in zip(
        corrected["environment_from_machine_root"]["orientation_wxyz"], [2**-.5, 0., 0., 2**-.5]))
    assert corrected["source"]["unconstrained_detection"]["machine_position"] == [1., 2., 3.]
    assert observations == fixture()[1]
    for bad in (float("nan"), "0.78", None):
        invalid = deepcopy(constrained_profile)
        invalid["scene_support"]["machine_root_z_m"] = bad
        rejects(lambda p: build_source(summary, tilted, p, "fixture"), invalid)
    assert scene["source"]["detection"]["providers"][0]["tracks"]["coffee_machine"] == observations[0]["tracks"][0]["provenance"]
    ambiguous_cup = deepcopy(observations)
    for row in ambiguous_cup:
        row["tracks"][1]["orientation_ambiguous"] = True
    assert build_source(summary, ambiguous_cup, profile, "fixture")["scene"]["environment_from_cup"] == scene["environment_from_cup"]
    rejects(ObservationWindow(profile, []).add, observations[0])  # HTTP default remains live-only.
    for change in ({"status": "FAILED"}, {"cleanup_complete": False}, {"valid_pair_span_s": 4.9},
                   {"valid_pair_frames": 3}, {"runtime_pack_digest": ""}):
        rejects(lambda s: build_source(s, observations, profile, "fixture"), {**summary, **change})
    for mutate in (
        lambda r: r.update(mode="live"),
        lambda r: r.update(source_stamp_ns=1_000_000_000),
        lambda r: r.update(tracking_session_id="changed"),
        lambda r: r.update(provider_status="FAILED"),
        lambda r: r["tracks"][0].update(pose_current=False),
        lambda r: r["tracks"][0].update(pose_current="True"),
        lambda r: r["tracks"][0].update(orientation_ambiguous=True),
        lambda r: r["tracks"][1].update(pose_current=False),
        lambda r: r["tracks"].append("malformed"),
        lambda r: r["tracks"][0]["world"].update(xyz_m=[1., float("nan"), 3.]),
        lambda r: r["tracks"][0]["world"].update(quaternion_xyzw=[0., 0., 0., 0.]),
        lambda r: r["tracks"][0]["provenance"].update(provider_id="ground_truth"),
        lambda r: r["tracks"][1]["provenance"].update(provider_id="online_detection.center_3d"),
    ):
        changed = deepcopy(observations)
        mutate(changed[1])
        rejects(lambda rows: build_source(summary, rows, profile, "fixture"), changed)
    short_summary, short_rows, _ = fixture()
    short_summary["inputs"][-1]["source_stamp_ns"] -= 1
    short_rows[-1]["source_stamp_ns"] -= 1
    rejects(lambda rows: build_source(short_summary, rows, profile, "fixture"), short_rows)
    # Keep the latest real 5s span, excluding early tracking startup and older valid frames.
    buffered_rows = []
    for i in range(10):
        row = deepcopy(observations[0])
        row.update(source_stamp_ns=1_000_000_000 + i * 375_000_000, source_sequence_id=i, capture_sequence_id=i,
                   pair_valid=i > 0)
        buffered_rows.append(row)
    buffered_summary = deepcopy(summary)
    buffered_summary.update(valid_pair_frames=9, valid_pair_span_s=3., inputs=[
        {"source_stamp_ns": r["source_stamp_ns"], "capture_sequence_id": r["capture_sequence_id"], "sha256": "a" * 64}
        for r in buffered_rows])
    latest = build_source(buffered_summary, buffered_rows, profile, "fixture")["scene"]["source"]
    assert latest["window_start_ns"] == 1_375_000_000 and latest["window_end_ns"] == 4_375_000_000
    assert latest["sample_count"] == 9 and len(latest["detection"]["providers"]) == 9
    buffered_rows[5]["pair_valid"] = False
    buffered_summary["valid_pair_frames"] = 8
    rejects(lambda rows: build_source(buffered_summary, rows, profile, "fixture"), buffered_rows)

    # Fake-real experiment: a cup move invalidates the old window, then the
    # bridge rebuilds the scene from the post-move five-second observation.
    moved_rows = []
    for i in range(13):
        row = deepcopy(observations[0])
        row.update(source_stamp_ns=1_000_000_000 + i * 375_000_000, source_sequence_id=i,
                   capture_sequence_id=i, pair_valid=True)
        row["tracks"][1]["world"]["xyz_m"] = [1.05 if i >= 3 else 1.0, 2.0, 3.0]
        moved_rows.append(row)
    moved_summary = deepcopy(summary)
    moved_summary.update(cup_motion_threshold_m=.001, valid_pair_frames=13, valid_pair_span_s=4.5, inputs=[
        {"source_stamp_ns": r["source_stamp_ns"], "capture_sequence_id": r["capture_sequence_id"],
         "sha256": "a" * 64} for r in moved_rows])
    moved_source = build_source(moved_summary, moved_rows, profile, "fake-cup-move")
    moved_detection = moved_source["scene"]["source"]["detection"]
    assert moved_detection["reobserve_count"] == 1
    assert moved_detection["reobserve_events"][0]["source_stamp_ns"] == 2_125_000_000
    assert moved_source["scene"]["source"]["window_start_ns"] == 2_500_000_000
    assert moved_source["scene"]["source"]["window_end_ns"] == 5_500_000_000
    assert moved_source["scene"]["environment_from_cup"]["position"] == [1.05, 2.0, 3.0]
    assert moved_source["scene"]["environment_from_machine_root"]["position"] == [1.0, 2.0, 3.0]
    assert moved_source["scene"]["environment_from_cup"]["orientation_wxyz"] == [1.0, 0.0, 0.0, 0.0]

    # A known noise threshold still rejects movement immediately. Unknown
    # noise bootstraps over >=5s/>=6 samples (test_observation_bootstrap.py).
    early_window = ObservationWindow(profile, [], mode="offline_replay", cup_motion_threshold_m=.001)
    early_move = None
    for i, offset in enumerate((0.0, 0.0, 0.05), 1):
        row = deepcopy(observations[0])
        row["source_stamp_ns"] = i * 1_000_000_000
        row["tracks"][1]["xyz_m"][0] += offset
        row["tracks"][1]["world"]["xyz_m"][0] += offset
        row["mode"] = "offline_replay"
        early_move = early_window.add(row)
    assert early_move["reobserve_required"] is True
    assert early_move["reobserve_event"]["threshold_source"] == "provided_calibration"
    assert early_window.status()["frame_count"] == 0

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "profile.json").write_text(json.dumps(profile))
        (root / "summary.json").write_text(json.dumps(summary))
        (root / "observations.jsonl").write_text("\n".join(json.dumps(r) for r in observations))
        result = convert(root / "profile.json", root, root / "output", "fixture")
        assert len(result["detection_files"]["summary_sha256"]) == 64
        assert json.loads((root / "output/suite.json").read_text()) == {"episodes": [scene["episode_scene"]]}
        from contract import demo_state
        from integration import _digest
        initial=demo_state();initial.update(left_gripper=71.,right_gripper=.7)
        metadata={'initial_robot_state':initial,'initial_robot_state_digest':_digest(initial),'gripper_mode':'distance'}
        (root/'initial-suite.json').write_text(json.dumps({'episodes':[{'metadata':metadata}]}))
        convert(root/'profile.json',root,root/'sim-initial','fixture',simulation_initial_suite=root/'initial-suite.json')
        assert json.loads((root/'sim-initial/suite.json').read_text())['episodes'][0]['metadata']==metadata
        metadata['initial_robot_state_digest']='wrong'
        (root/'initial-suite.json').write_text(json.dumps({'episodes':[{'metadata':metadata}]}))
        rejects(lambda path:convert(root/'profile.json',root,path,'fixture',simulation_initial_suite=root/'initial-suite.json'),root/'invalid-initial')
        assert not (root/'invalid-initial').exists()
        try:
            convert(root / "profile.json", root, root / "output", "fixture")
        except FileExistsError:
            pass
        else:
            raise AssertionError("existing output overwritten")
    print("offline detection-to-source CPU boundary check: PASS (no inference)")


if __name__ == "__main__":
    main()
