from copy import deepcopy

from contract import demo_state, validate_boundary, validate_command, validate_physical14
from integration import DualGpuPipeline, ObservationWindow, PIPELINE_LANES, _digest, freeze_receipt, validate_profile


PROFILE = {
    "schema_version": "calibration_profile.v1", "profile_id": "fixture-v1", "source": {"camera": "d405", "joints": "fixture"},
    "camera": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    "joints": {"zero_offsets_rad": [0.0] * 12, "signs": [1.0] * 12},
    "gripper": {"mapping": "total_opening_mm", "max_opening_mm": 88.0},
    "frames": {"environment_from_camera": {"position": [0.0, 0.0, 0.0], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}},
    "divergence_thresholds": {"joint_rad": 0.05, "gripper_mm": 2.0, "timing_s": 0.02},
}


def snapshot(stamp):
    return {"mode": "live", "source_stamp_ns": stamp, "frame_id": "camera_color_optical_frame", "tracks": [
        {"track_id": "machine", "object_type": "coffee_machine", "pose_current": True,
         "xyz_m": [0.4, 0.1, 0.2], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        {"track_id": "cup", "object_type": "cup", "pose_current": True,
         "xyz_m": [0.5, 0.1, 0.8]},
    ]}


def main() -> None:
    state = demo_state()
    assert validate_physical14(state) is state
    command = {"action": state, "dry_run": True}
    assert validate_command(command) is command
    validate_boundary("calibration", {"result_id": "r1", "source": "fixture", "status": "ready"})
    validate_boundary("online_detection", {"timestamp_s": 1.0, "objects": [{"label": "cup"}]})
    profile = validate_profile(PROFILE)
    window = ObservationWindow(profile, [])
    for i in range(6):
        window.add(snapshot((i + 1) * 1_000_000_000))
    scene = window.finalize()
    assert scene["environment_from_cup"]["orientation_wxyz"] == [1.0, 0.0, 0.0, 0.0]
    receipt = freeze_receipt({"source_revision": "fixture", "ticks": [
        {"tick": 0, "time_s": 0.0, "target": state}, {"tick": 1, "time_s": 0.1, "target": state}],
        "simulation": {"validated": False}}, profile, scene)
    assert receipt["state"] == "awaiting_sim_validation" and receipt["receipt_digest"]
    check_pipeline(scene, receipt, state)
    for bad in (
        {**state, "left_arm_joints": [0.0]},
        {**state, "encoding": "delta"},
        {**state, "padded32": [0.0] * 32},
    ):
        try:
            validate_physical14(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid payload: {bad}")
    print("physical14 and adapter boundary harness: PASS")


def rejects(call, payload):
    try:
        call(payload)
    except ValueError:
        return
    raise AssertionError(f"accepted invalid payload: {payload}")


def check_pipeline(scene, receipt, state):
    bridge = DualGpuPipeline()
    source = {"schema_version": "dual_gpu_pipeline.v1", **PIPELINE_LANES["simulated-real"],
              "lane": "simulated-real", "adapter": "sim-online-detection-ground-truth", "sequence": 0,
              "run_id": "source-test", "scene": scene, "calibration_profile": PROFILE}
    rejects(bridge.ingest_source, {**source, "instance_id": PIPELINE_LANES["dsw-scene"]["instance_id"]})
    rejects(bridge.ingest_source, {**source, "gpu": False})
    tampered = deepcopy(source)
    tampered["scene"]["episode_scene"]["seed"] += 1
    rejects(bridge.ingest_source, tampered)
    tampered["scene"]["scene_digest"] = _digest({k: v for k, v in tampered["scene"].items() if k != "scene_digest"})
    rejects(bridge.ingest_source, tampered)  # Fresh digest cannot hide inconsistent episode_scene.
    bridge.ingest_source(source)
    references = {"source_run_id": source["run_id"], "source_sequence": 0, "scene_digest": scene["scene_digest"],
                  "calibration_profile_digest": _digest(PROFILE)}
    plan = {"schema_version": "dual_gpu_pipeline.v1", **PIPELINE_LANES["dsw-scene"], **references,
            "lane": "dsw-scene", "run_id": "plan-test", "planner_id": "coffee_episode_relative_rule_002",
            "policy_id": "coffee_relative_rule_development_004", "timeline_digest": receipt["timeline_digest"],
            "ticks": receipt["ticks"]}
    for change in ({"timeline_digest": "not-a-digest"}, {"source_sequence": 1}, {"gpu": 1},
                   {"calibration_profile_digest": "other-profile"}, {"source_run_id": "other-source"}):
        rejects(bridge.accept_plan, {**plan, **change})
    for field, value in (("time_s", float("nan")), ("time_s", -1), ("time_s", 0), ("tick", True), ("tick", 3),
                         ("target", {**state, "left_gripper": 100}), ("target", {**state, "encoding": "delta"})):
        invalid = deepcopy(plan)
        invalid["ticks"][1][field] = value
        rejects(bridge.accept_plan, invalid)
    bridge.accept_plan(plan)
    execution = {"schema_version": "dual_gpu_pipeline.v1", **references, "plan_run_id": plan["run_id"],
                 "timeline_digest": plan["timeline_digest"], "status": "completed", "tick_count": 2,
                 "applied_ticks": plan["ticks"], "measured_states": [state, state]}
    def lane(side, **extra):
        return {**execution, "side": side, **PIPELINE_LANES[side], **extra}
    for change in ({"tick_count": 1}, {"tick_count": True}, {"applied_ticks": plan["ticks"][:1]},
                   {"measured_states": []}, {"status": "running"}, {"plan_run_id": "stale-plan"},
                   {"timeline_digest": "forged"}, {"scene_digest": "stale-scene"}):
        rejects(bridge.record_execution, lane("simulated-real", **change))
    invalid = deepcopy(lane("simulated-real"))
    invalid["applied_ticks"][1]["target"]["left_gripper"] += 1
    rejects(bridge.record_execution, invalid)
    bridge.record_execution(lane("simulated-real"))
    summary = bridge.record_execution(lane("dsw-scene"))
    assert summary["alignment"] == "aligned" and summary["execution"] == "replay-complete"
    assert summary["timing"]["status"] == "unverified"
    assert summary["scene_comparison"]["status"] == "unverified"
    actual = deepcopy(scene["episode_scene"])
    actual["environment_from_cup"]["position"][0] += 0.01
    actual["environment_from_machine_root"]["orientation_wxyz"] = [2**-0.5, 0, 0, 2**-0.5]
    def actual_scene(side, value):
        return lane(side, actual_episode_scene=value, actual_episode_digest=_digest(value),
                    detected_scene_digest=scene["scene_digest"])
    rejects(bridge.record_execution, actual_scene("dsw-scene", actual))
    rejects(bridge.record_execution, {**actual_scene("simulated-real", actual), "actual_episode_digest": "forged"})
    bridge.record_execution(actual_scene("simulated-real", actual))
    audit = bridge.record_execution(actual_scene("dsw-scene", scene["episode_scene"]))["scene_comparison"]
    assert abs(audit["cup_position_error_m"] - 0.01) < 1e-10
    import math
    assert abs(audit["machine_rotation_error_rad"] - math.pi/2) < 1e-10
    measured = {key: deepcopy(actual[key]) for key in ("environment_from_machine_root", "environment_from_cup")}
    rejects(bridge.record_execution, lane("simulated-real", measured_reset_scene=measured))
    bridge.record_execution(lane("simulated-real", measured_reset_scene=measured,
                                 measured_scene_source="isaac_post_reset_mesh_roots"))
    reconstructed = deepcopy(measured)
    reconstructed["environment_from_cup"]["position"][2] += .003
    # Cup orientation is deliberately irrelevant to reconstruction error.
    reconstructed["environment_from_cup"]["orientation_wxyz"] = [0, 0, 0, 1]
    audit = bridge.record_execution(lane("dsw-scene", measured_reset_scene=reconstructed,
        measured_scene_source="isaac_post_reset_mesh_roots"))["measured_scene_comparison"]
    assert audit["status"] == "measured" and abs(audit["cup_position_error_m"] - .003) < 1e-10
    assert audit["machine_rotation_error_rad"] < 1e-7 and "cup_rotation_error_rad" not in audit
    reconstructed["environment_from_cup"]["position"][0] = float("nan")
    rejects(bridge.record_execution, lane("dsw-scene", measured_reset_scene=reconstructed,
                                          measured_scene_source="isaac_post_reset_mesh_roots"))
    def timing(stamps, uncertainty=0.001):
        return {"clock": "utc", "uncertainty_s": uncertainty, "evidence": "fixture-clock-measurement.json",
                "applied_at_unix_s": stamps}
    rejects(bridge.record_execution, lane("simulated-real", timing=timing([1000, 999])))
    rejects(bridge.record_execution, lane("simulated-real", timing=timing([1000, 1000.1], -1)))
    bridge.record_execution(lane("simulated-real", timing=timing([1000, 1000.1])))
    summary = bridge.record_execution(lane("dsw-scene", timing=timing([1000.003, 1000.103])))
    assert summary["execution"] == "synchronized" and summary["timing"]["status"] == "passed"
    summary = bridge.record_execution(lane("dsw-scene", timing=timing([1100, 1100.1])))
    assert summary["execution"] == "replay-complete" and summary["timing"]["status"] == "exceeded"
    bridge.record_execution(lane("simulated-real", timing=timing([1000, 1001])))
    summary = bridge.record_execution(lane("dsw-scene", timing=timing([1000, 1001])))
    assert summary["execution"] == "replay-complete"  # Barrier together, but not at timeline cadence.
    bridge.record_execution(lane("simulated-real", timing=timing([1000, 1000.1], 0.02)))
    summary = bridge.record_execution(lane("dsw-scene", timing=timing([1000, 1000.1], 0.02)))
    assert summary["execution"] == "replay-complete"  # Uncertain clocks cannot certify synchronization.
    assert bridge.ingest_source(source)["execution"] == "replay-complete"  # Idempotent retry.
    assert bridge.accept_plan(plan)["execution"] == "replay-complete"
    plan["ticks"][0]["time_s"] = 99
    assert bridge.plan["ticks"][0]["time_s"] == 0  # Caller cannot mutate the frozen timeline.
    assert bridge.ingest_source({**source, "sequence": 1})["execution"] == "waiting"
    rejects(bridge.accept_plan, plan)  # A new observation invalidates the previous plan.


if __name__ == "__main__":
    main()
