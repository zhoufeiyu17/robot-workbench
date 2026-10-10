"""Freeze a completed native EpisodePlanner/RuleCoffeePolicy rollout for both lanes."""
import argparse
import hashlib
import json
from real_contract import is_device_source
from pathlib import Path

from integration import CUROBO_PLANNER_ID, DualGpuPipeline, PIPELINE_LANES, SIM_PLANNER_ID, SIM_POLICY_ID, _digest
from isaac_lane import physical, write


def export(source_path, episode_path, output, hardware_binding=None, *, environment='physical'):
    from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
    spec = ac_one_dual_arm_zero_home_action_spec()
    source = json.loads(source_path.read_text())
    report = json.loads((episode_path / "report.json").read_text())
    if report.get("outcome") != "success":
        raise ValueError("native rollout did not succeed")
    if report["identity"]["planner_id"] != SIM_PLANNER_ID:
        raise ValueError("native planner identity mismatch")
    if any(report["requested_scene"].get(k) != v for k, v in source["scene"]["episode_scene"].items()):
        raise ValueError("native rollout used a different observed scene")
    if 'initial_state' in source:
        model = source['initial_state']['model']
        metadata = report['requested_scene'].get('metadata') or {}
        if metadata.get('initial_robot_state') != model or metadata.get('initial_robot_state_digest') != _digest(model):
            raise ValueError('native rollout did not use the sealed model initial state')
    control = report["identity"]["control"]
    opening_mm = report["identity"]["acceptance"]["open_position_m"] * 2000
    if not 0 < opening_mm <= source["calibration_profile"]["gripper"]["max_opening_mm"]:
        raise ValueError("native gripper opening exceeds calibration profile")
    if opening_mm != 88.0 and report["identity"].get("effective_gripper_opening_mm") != opening_mm:
        raise ValueError("non-default opening requires native action/reset/planner configuration evidence")
    dt = control["substeps"] / control["physics_hz"]
    ticks, actions, stages, substeps = [], [], [], []
    lines = (episode_path / "steps.jsonl").read_text().splitlines()
    for i, line in enumerate(lines):
        step = json.loads(line)
        receipt = step["info"]["execution_report"]
        terminal_success = (i == len(lines) - 1 and receipt["terminated"]
                            and (receipt.get("event") or {}).get("reason") == "success")
        count = receipt["actual_substeps"]
        if (receipt["action_applied"] is not True or not 0 < count <= control["substeps"]
                or receipt.get("truncated") or (receipt["terminated"] and not terminal_success)
                or (i == len(lines) - 1 and not terminal_success)
                or (count != control["substeps"] and not terminal_success)):
            raise ValueError("rollout contains rejected command")
        substeps.append(count)
        action = receipt["applied_action"]
        ticks.append({"tick": i, "time_s": i * dt, "target": physical(spec.decode(action), opening_mm)})
        actions.append(action)
        if not stages or stages[-1]["name"] != step["stage"]:
            stages.append({"name": step["stage"], "start_tick": i, "start_s": i * dt})
    plan = {"schema_version": "dual_gpu_pipeline.v1", **PIPELINE_LANES["dsw-scene"],
        "lane": "dsw-scene", "run_id": output.parent.name + "-plan",
        "source_run_id": source["run_id"], "source_sequence": source["sequence"],
        "scene_digest": source["scene"]["scene_digest"],
        "calibration_profile_digest": _digest(source["calibration_profile"]),
        "planner_id": SIM_PLANNER_ID, "policy_id": SIM_POLICY_ID,
        "ticks": ticks, "timeline_digest": _digest(ticks), "native_actions": actions,
        "native_substeps": substeps, "duration_s": sum(substeps) / control["physics_hz"],
        "gripper_opening_mm": opening_mm,
        "control": control, "motion_profile": report["identity"]["motion_profile"],
        "stages": stages, "native_report": str(episode_path / "report.json"),
        "native_report_sha256": hashlib.sha256((episode_path / "report.json").read_bytes()).hexdigest(),
        "native_identity": report["identity"],
        "simulation": {"validated": True, "outcome": report["outcome"]}}
    from joint_space import freeze_reported
    freeze_reported(plan, source)
    if 'initial_state' in source:
        plan['initial_state_digest'] = _digest(source['initial_state'])
    if is_device_source(source):
        from gripper_distance_plan import read_trace, distance_plan
        plan = distance_plan(plan, source, read_trace(episode_path, plan))
    summary = validate_export(plan, source, hardware_binding, environment=environment)
    write(output, plan)
    print(json.dumps(summary))


def validate_export(plan, source, hardware_binding=None, *, environment='physical'):
    if is_device_source(source):
        if hardware_binding is None:
            raise ValueError("physical plan export requires trusted hardware binding")
        from real_contract import validate_real_plan
        binding = json.loads(Path(hardware_binding).read_text())
        validate_real_plan(plan, source, binding, environment=environment)
        return {"mode": "fake_device" if environment=='fake' else "real", "source_run_id": source["run_id"],
                "timeline_digest": plan["timeline_digest"], "tick_count": len(plan["ticks"])}
    if hardware_binding is not None:
        raise ValueError("physical hardware binding cannot authorize simulated source")
    pipeline = DualGpuPipeline()
    pipeline.ingest_source(source)
    pipeline.accept_plan(plan)
    return pipeline.summary()


def publish_motiongen(source_path, planned_directory, output, cup_contact_preset=None):
    """Freeze the single generated dual-DSW plan; never replan or rewrite targets."""
    source = json.loads(Path(source_path).read_text())
    directory = Path(planned_directory)
    plan = json.loads((directory/'plan.json').read_text())
    result = json.loads((directory/'result.json').read_text())
    if (source.get('schema_version') != 'dual_gpu_pipeline.v1' or source.get('lane') != 'simulated-real'
            or plan.get('planner_id') != CUROBO_PLANNER_ID or result.get('status') != 'motiongen-plan-ready'
            or result.get('backend') != 'curobo_motiongen' or result.get('timeline_digest') != plan.get('timeline_digest')):
        raise ValueError('completed MotionGen result and simulated source required for dual replay')
    bridge = DualGpuPipeline()
    bridge.ingest_source(source)
    bridge.accept_plan(plan)
    if cup_contact_preset is not None:
        from cup_presets import bind_plan_preset
        bind_plan_preset(plan,cup_contact_preset)
        bridge.accept_plan(plan)
    write(output, plan)
    return dict(status='motiongen-plan-published', timeline_digest=plan['timeline_digest'],
                tick_count=len(plan['ticks']), source_run_id=source['run_id'])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ("source", "episode", "output"):
        parser.add_argument("--" + field, type=Path, required=True)
    parser.add_argument("--hardware-binding", type=Path)
    parser.add_argument('--environment',choices=('physical','fake'),default='physical')
    args = parser.parse_args()
    export(args.source, args.episode, args.output, args.hardware_binding,environment=args.environment)
