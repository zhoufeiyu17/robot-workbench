"""Run HTTP and durable-state boundary checks with isolated, temporary storage."""

from contextlib import contextmanager
from copy import deepcopy
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
import hashlib
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread
from types import SimpleNamespace
from unittest.mock import patch, Mock

import app
from contract import demo_state
from integration import ObservationWindow, PIPELINE_LANES, _digest, freeze_receipt
from test_harness import PROFILE, snapshot


def fixture_messages():
    profile = deepcopy(PROFILE)
    window = ObservationWindow(profile, [])
    for stamp in range(1_000_000_000, 7_000_000_000, 1_000_000_000):
        window.add(snapshot(stamp))
    scene = window.finalize()
    ticks = [{"tick": i, "time_s": i * 0.1, "target": demo_state()} for i in range(2)]
    receipt = freeze_receipt({"ticks": ticks, "simulation": {"validated": False}}, profile, scene)
    source = {"schema_version": "dual_gpu_pipeline.v1", **PIPELINE_LANES["simulated-real"],
              "lane": "simulated-real", "adapter": "sim-online-detection-ground-truth",
              "sequence": 0, "run_id": "http-source", "scene": scene, "calibration_profile": profile}
    reference = {"source_run_id": source["run_id"], "source_sequence": source["sequence"],
                 "scene_digest": scene["scene_digest"], "calibration_profile_digest": _digest(profile)}
    plan = {"schema_version": "dual_gpu_pipeline.v1", **PIPELINE_LANES["dsw-scene"], **reference,
            "lane": "dsw-scene", "run_id": "http-plan", "planner_id": receipt["planner_id"],
            "policy_id": receipt["policy_id"], "ticks": ticks, "timeline_digest": receipt["timeline_digest"]}
    executions = []
    for side in PIPELINE_LANES:
        measured = [demo_state() for _ in ticks]
        if side == "dsw-scene":
            for state in measured:
                state["left_arm_joints"] = [0.012] * 6
                state["right_gripper"] += 1.5
        executions.append({"schema_version": "dual_gpu_pipeline.v1", **PIPELINE_LANES[side], **reference,
                           "side": side, "plan_run_id": plan["run_id"], "timeline_digest": plan["timeline_digest"],
                           "status": "completed", "tick_count": len(ticks), "applied_ticks": ticks,
                           "measured_states": measured})
    return source, plan, executions


@contextmanager
def serving(state):
    previous = app.STATE
    app.STATE = state
    server = ThreadingHTTPServer(("127.0.0.1", 0), app.Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def request(method, path, payload=None, expected=200, headers=None, response_headers=False):
        connection = HTTPConnection(*server.server_address, timeout=5)
        try:
            body = json.dumps(payload) if payload is not None else None
            connection.request(method, path, body=body, headers={"Content-Type": "application/json", **(headers or {})})
            response = connection.getresponse()
            raw = response.read()
            assert response.status == expected, (path, response.status, raw)
            assert response.getheader("Cache-Control") == "no-store"
            if response_headers:
                return raw, dict(response.getheaders())
            return json.loads(raw) if raw and (response.getheader("Content-Type") or "").startswith("application/json") else raw
        finally:
            connection.close()

    try:
        yield request
    finally:
        server.shutdown()
        thread.join(timeout=5)
        server.server_close()
        app.STATE = previous
        assert not thread.is_alive(), "test server did not stop"


def main():
    source, plan, executions = fixture_messages()
    with TemporaryDirectory(prefix="rule-workbench-http-") as folder:
        directory = Path(folder)
        state = app.WorkbenchState()
        state.restore(directory)
        with serving(state) as request:
            assert request("GET", "/api/state")["pipeline"]["execution"] == "waiting"
            request("POST", "/api/pipeline/source", [], expected=400)
            request("POST", "/api/pipeline/plan", plan, expected=400)
            request("POST", "/api/pipeline/source", source)
            assert request("GET", "/api/calibration/profile") == source["calibration_profile"]
            assert request("GET", "/api/scene") == source["scene"]
            request("POST", "/api/pipeline/plan", {**plan, "timeline_digest": "forged"}, expected=400)
            assert request("GET", "/api/pipeline/status")["tick_count"] == 0
            request("POST", "/api/pipeline/plan", plan)
            first = request("POST", "/api/pipeline/execution", executions[0])
            assert first["execution"] == "in-progress"
            request("POST", "/api/pipeline/execution", {**executions[1], "tick_count": 1}, expected=400)
            second = request("POST", "/api/pipeline/execution", executions[1])
            assert second["execution"] == "replay-complete", "two receipts cannot prove clock synchronization"
            (directory / "source.png").write_bytes(b"test-image")
            (directory / "target.png").write_bytes(b"test-image")
            request("GET", "/api/evidence/source.png", expected=404)  # Unbound historical images are hidden.
            image_ref = {"source_run_id": source["run_id"], "source_sequence": source["sequence"],
                         "scene_digest": source["scene"]["scene_digest"], "plan_run_id": plan["run_id"]}
            (directory / "evidence.json").write_text(json.dumps({"images": {name: image_ref for name in ("source.png", "target.png")}}))
            assert request("GET", "/api/evidence/source.png?t=current") == b"test-image"
            assert request("GET", "/api/evidence/target.png?t=current") == b"test-image"
            before_restart = request("GET", "/api/state")
            comparison = before_restart["pipeline"]["comparison"]
            assert math.isclose(comparison["max_joint_error_rad"], 0.012)
            assert math.isclose(comparison["max_gripper_error_mm"], 1.5)
            assert before_restart["state"] == executions[1]["measured_states"][-1]
            persisted_before = (directory / "state.json").read_bytes()
            denied = request("POST", "/api/command", {"action": demo_state(), "dry_run": False}, expected=409)
            assert denied["error"] == "hardware_required"
            assert (directory / "state.json").read_bytes() == persisted_before
            assert request("GET", "/api/state")["state"] == before_restart["state"]
            assert not (directory / "state.json.partial").exists()

        # A fresh state object and fresh HTTP server must recover the actual persisted pipeline.
        restored = app.WorkbenchState()
        restored.restore(directory)
        with serving(restored) as request:
            after_restart = request("GET", "/api/state")
            assert after_restart == before_restart, "restart changed the full state summary"
            assert state.window is None and restored.window is None
            assert restored.pipeline.source == source and restored.pipeline.plan == plan
            assert restored.pipeline.executions == {item["side"]: item for item in executions}
            changed_profile = deepcopy(PROFILE)
            changed_profile["profile_id"] = "fixture-v2"
            changed_profile["joints"]["zero_offsets_rad"][0] = 0.01
            changed_profile["gripper"]["max_opening_mm"] = 83.0
            request("POST", "/api/calibration", {"profile": changed_profile})
            request("GET", "/api/evidence/source.png?t=stale", expected=404)
            request("GET", "/api/evidence/target.png?t=stale", expected=404)
            reset = request("GET", "/api/state")
            assert reset["calibration_profile"]["profile_id"] == "fixture-v2"
            assert reset["pipeline"]["execution"] == "waiting" and reset["pipeline"]["tick_count"] == 0
            assert reset["pipeline"]["execution_lanes"] == []
            assert reset["scene"] is None and reset["trajectory"] is None
            request("POST", "/api/pipeline/plan", plan, expected=400)
            request("POST", "/api/pipeline/execution", executions[0], expected=400)
            request("POST", "/api/command", {"action": {**demo_state(), "left_gripper": 88},
                                               "dry_run": True}, expected=400)
            result = request("POST", "/api/command", {"action": demo_state(), "dry_run": True})
            assert result["dry_run"] is True and result["adapter"] == "workbench-dry-run"

        final = app.WorkbenchState()
        final.restore(directory)
        assert final.profile["profile_id"] == "fixture-v2" and final.pipeline.plan is None
        assert final.pipeline.source is None and not final.pipeline.executions
        assert final.last_action == demo_state()
        assert final.window is not None and final.window.status()["frame_count"] == 0
        with serving(final) as request:
            partial = request("POST", "/api/online-detection", {"snapshot": snapshot(1_000_000_000)})
            assert partial == {"frame_count": 1, "span_s": 0.0, "ready": False}
            window_before_restart = request("GET", "/api/state")
        resumed_window = app.WorkbenchState()
        resumed_window.restore(directory)
        assert resumed_window.window is not None
        with serving(resumed_window) as request:
            assert request("GET", "/api/state") == window_before_restart, "restart changed active observation window"
            for stamp in range(2, 7):
                completed = request("POST", "/api/online-detection", {"snapshot": snapshot(stamp * 1_000_000_000)})
            assert completed["ready"] is True and completed["frame_count"] == 6 and completed["span_s"] == 5
    print("HTTP pipeline, durable restart, calibration invalidation and hardware guard: PASS")
    check_cup_move_reobserve()
    check_job(source, plan, executions)


def check_cup_move_reobserve():
    """Fake-real cup motion clears the old scene/plan and rebuilds the window."""
    source, plan, _ = fixture_messages()
    with TemporaryDirectory(prefix="rule-workbench-cup-move-") as folder:
        state = app.WorkbenchState()
        state.restore(Path(folder))
        state.profile = deepcopy(PROFILE)
        state.window = ObservationWindow(state.profile, [])
        state.scene = source["scene"]
        state.pipeline.ingest_source(source)
        state.pipeline.accept_plan(plan)
        state.receipt = {"state": "sim_validated", "scene_digest": source["scene"]["scene_digest"]}

        def cup_snapshot(stamp, moved=False):
            row = snapshot(stamp)
            if moved:
                row["tracks"][1]["xyz_m"][0] += 0.05
            return row

        with serving(state) as request:
            for stamp in range(1, 7):
                request("POST", "/api/online-detection", {"snapshot": cup_snapshot(stamp * 1_000_000_000)})
            moved = request("POST", "/api/online-detection", {
                "snapshot": cup_snapshot(7_000_000_000, moved=True)})
            assert moved["reobserve_required"] is True and moved["reobserve_count"] == 1
            assert moved["scene"] is None and state.scene is None
            assert state.pipeline.plan is None and state.pipeline.source is None
            for stamp in range(8, 14):
                completed = request("POST", "/api/online-detection", {
                    "snapshot": cup_snapshot(stamp * 1_000_000_000, moved=True)})
            assert completed["ready"] is True and completed["reobserve_count"] == 1
            assert completed["scene"]["environment_from_cup"]["position"][0] == 0.55
            assert state.pipeline.plan is None, "target plan must be rebuilt from the new scene"
    print("Fake-real cup movement reobserve + scene/plan invalidation: PASS")


def check_job(source, plan, executions):
    with TemporaryDirectory(prefix="rule-workbench-job-") as folder:
        root = Path(folder).resolve()
        config = root / "operator-config.json"
        contract_path = root / "operator-contract.json"
        contract = {"filesystem": {"readonly_roots": ["/inputs"], "writable_roots": ["/runs/template", "/services/detector"],
                                   "allow_overwrite": False, "allow_delete": False},
                    "resources": {"max_gpu_count_per_instance": 1},
                    "detector_service": {"root": "/services/detector", "socket": "/services/detector/service.sock",
                                         "owner_token": "test-owned-session", "identity": "pinned-model-identity",
                                         "allowed_read_roots": ["/inputs"], "allowed_output_root": "/runs",
                                         "max_memory_mib": 2048, "min_free_memory_mib": 1024,
                                         "max_lifetime_seconds": 600},
                    "provenance": {"config_sha256": {"config.json": "stale"},
                                   "input_sha256": {"/inputs/suite.json": "a" * 64},
                                   "code_sha256": {"/inputs/policy.py": "b" * 64}}}
        contract_path.write_text(json.dumps(contract))
        settings = {"remote_run_root": "/runs/template", "contract_path": str(contract_path),
                    "workbench_code": "/inputs/workbench", "record_raw": True,
                    "detector_service": deepcopy(contract["detector_service"]),
                    "motion_profile": "continuous-target", "opening_mm": 83}
        config.write_text(json.dumps(settings))
        template_bytes, contract_bytes = config.read_bytes(), contract_path.read_bytes()
        state = app.WorkbenchState()
        state.restore(Path(os.path.relpath(root)))
        assert state.directory == root and state.directory.is_absolute()
        initialization = {"status": "starting", "stage": "loading", "message": "正在加载检测模型",
                          "progress_percent": 40, "elapsed_s": 65, "eta_s": 0}
        detector = SimpleNamespace(status=lambda: dict(initialization))
        with serving(state) as request, patch.object(app, "PIPELINE_CONFIG", config), patch.object(app, "DETECTOR_MANAGER", detector):
            with patch.object(app, "Thread") as worker:
                request("POST", "/api/pipeline/run", {}, expected=403, headers={"Origin": "https://untrusted.example"})
                request("POST", "/api/pipeline/run", {"command": "untrusted"}, expected=400)
                assert request("GET", "/api/initialization") == initialization
                for route in ("/api/pipeline/run", "/api/pipeline/diagnostic"):
                    denied = request("POST", route, {}, expected=503)
                    assert denied["initialization"]["status"] == "starting"
                initialization.update(status="failed", error="model load failed")
                assert "model load failed" in request("POST", "/api/pipeline/run", {}, expected=503)["error"]
                for status in ("busy", "stopped"):
                    initialization.update(status=status, error=None)
                    request("POST", "/api/pipeline/run", {}, expected=503)
                assert state.job is None and not (root / "jobs").exists()
                worker.return_value.start.assert_not_called()
                initialization.update(status="ready", progress_percent=100, eta_s=0, error=None)
                job = request("POST", "/api/pipeline/run", {}, expected=202)
                worker.return_value.start.assert_called_once()
                request("POST", "/api/pipeline/run", {}, expected=409)
                request("POST", "/api/calibration", {"profile": source["calibration_profile"]}, expected=409)
                remote_roots = set()
                previous_root = None
                for index, route in enumerate((None, "/api/pipeline/run", "/api/pipeline/diagnostic", "/api/pipeline/diagnostic")):
                    if route:
                        state.job["status"] = "failed"
                        if route.endswith("/diagnostic"):
                            historical = Path(state.job["directory"]) / "config.json"
                            old_config = json.loads(historical.read_text())
                            old_config["workbench_code"] = "/retired/forced-workbench"
                            historical.write_text(json.dumps(old_config))
                        with patch.object(state, "latest_visual_plan", return_value=plan):
                            current_job = request("POST", route, {}, expected=202)
                    else:
                        current_job = job
                    generated_path = Path(current_job["directory"]) / "config.json"
                    generated = json.loads(generated_path.read_text())
                    derived = json.loads(Path(generated["contract_path"]).read_text())
                    remote_root = generated["remote_run_root"]
                    assert remote_root != settings["remote_run_root"] and remote_root not in remote_roots
                    remote_roots.add(remote_root)
                    assert derived["filesystem"]["writable_roots"] == [remote_root, "/services/detector"]
                    assert derived["detector_service"] == contract["detector_service"]
                    assert generated["detector_service"] == derived["detector_service"]
                    assert derived["resources"] == contract["resources"]
                    for key in ("input_sha256", "code_sha256"):
                        assert derived["provenance"][key] == contract["provenance"][key]
                    assert derived["provenance"]["config_sha256"] == {
                        "config.json": hashlib.sha256(generated_path.read_bytes()).hexdigest()}
                    if index < 2:
                        assert generated["record_raw"] is True and "force_diagnostic" not in generated
                        assert generated["workbench_code"] == settings["workbench_code"]
                    else:
                        assert generated["force_diagnostic"] is True and generated["record_raw"] is False
                        assert generated["workbench_code"] == settings["workbench_code"], "diagnostic must use current pinned workbench code"
                        assert generated["diagnostic_input_root"] == previous_root
                        assert previous_root in derived["filesystem"]["readonly_roots"]
                    assert generated["opening_mm"] == settings["opening_mm"]
                    assert generated["motion_profile"] == settings["motion_profile"]
                    previous_root = remote_root
                assert config.read_bytes() == template_bytes and contract_path.read_bytes() == contract_bytes
                state.job = job
                state.persist()
            restarted = app.WorkbenchState()
            restarted.restore(root)
            assert restarted.job["status"] == "needs-review", "restart must not restart remote GPU work"
            output = Path(job["directory"])
            with patch.object(app.subprocess, "run", return_value=SimpleNamespace(returncode=1)):
                app.run_pipeline_job(output / "config.json", output)
            assert state.job["status"] == "failed" and state.pipeline.source is None
            good = root / "fixture-success"
            good.mkdir()
            def fake_pipeline(argv, **kwargs):
                artifacts = Path(argv[-1])
                artifacts.mkdir()
                for name, item in (("source.json", {**source, "adapter": "foundationpose"}),
                                   ("plan.json", plan), ("execution-source.json", executions[0]),
                                   ("execution-target.json", executions[1]), ("final.json", {"status": "completed"})):
                    (artifacts / name).write_text(json.dumps(item))
                for name in ("source.png", "target.png"):
                    (artifacts / name).write_bytes(b"CPU-fixture-image")
                return SimpleNamespace(returncode=0)
            with patch.object(app.subprocess, "run", side_effect=fake_pipeline):
                app.run_pipeline_job(config, good)
            assert state.job["status"] == "completed"
            assert request("GET", "/api/pipeline/status")["execution"] == "replay-complete"
            assert request("GET", "/api/evidence/source.png") == b"CPU-fixture-image"
            bad = root / "fixture-duplicate-lane"
            bad.mkdir()
            def duplicate_lane(argv, **kwargs):
                result = fake_pipeline(argv, **kwargs)
                artifacts = Path(argv[-1])
                (artifacts / "execution-target.json").write_text(json.dumps(executions[0]))
                return result
            previous = state.pipeline
            with patch.object(app.subprocess, "run", side_effect=duplicate_lane):
                app.run_pipeline_job(config, bad)
            assert state.job["status"] == "failed" and state.pipeline is previous
    print("Pipeline job admission, concurrent mutation guard, failure and atomic promotion: PASS (CPU fixtures)")


# Regression guard: a newer failed visual run must not inherit a promoted replay.
def check_failed_visual_run_is_current():
    with TemporaryDirectory(prefix="rule-workbench-failed-visual-") as folder:
        root = Path(folder)
        source, plan, _ = fixture_messages()
        job = root / "jobs" / "failed" / "result" / ".artifacts"
        job.mkdir(parents=True)
        ticks = [{"tick": 0, "time_s": 0.1, "target": demo_state()}]
        (job / "plan.json").write_text("DSW banner\\n" + json.dumps({"run_id": "visual-plan", "timeline_digest": "digest", "ticks": ticks}))
        (root / "state.json").write_text(json.dumps({"pipeline": {"source": source, "plan": plan, "executions": {}}, "job": {"status": "failed", "directory": str(root / "jobs" / "failed")}, "last_action": demo_state()}))
        (root / "source.png").write_bytes(b"historical")
        (root / "target.png").write_bytes(b"historical")
        reference = {"source_run_id": source["run_id"], "source_sequence": source["sequence"],
                     "scene_digest": source["scene"]["scene_digest"], "plan_run_id": plan["run_id"]}
        (root / "evidence.json").write_text(json.dumps({"status": "failed", "visual_pipeline": {"run_id": "visual-failed", "status": "failed", "detection": {"status": "passed", "provider_id": "foundationpose"}, "planning": {"status": "passed"}, "execution": {"status": "failed"}}, "images": {name: reference for name in ("source.png", "target.png")}}))
        state = app.WorkbenchState(); state.restore(root)
        current = state.summary()["current_run"]
        assert current["run_id"] == "failed" and current["promoted"] is False
        assert current["plan_available"] and current["plan"]["validated"] is False
        view = state.summary()
        assert not view["pipeline"]["execution_lanes"] and not view["pipeline"]["recording"]
        assert view["pipeline"]["tick_count"] == 1 and view["pipeline"]["timeline_digest"] == "digest"
        assert view["calibration_profile"] is None and view["scene"] is None
        assert view["pipeline"]["source_lane"]["status"] == "awaiting-observation"
        (job / "plan.json").unlink()
        empty = state.summary()
        assert not empty["current_run"]["plan_available"] and empty["pipeline"]["tick_count"] == 0
        assert empty["pipeline"]["timeline_digest"] is None
        (job / "plan.json").write_text(json.dumps({"run_id": "visual-plan", "timeline_digest": "digest", "ticks": ticks}))
        assert state.image_evidence("source.png") is None
        assert state.image_evidence("target.png") is None
        (root / "failed-source.png").write_bytes(b"failed-camera")
        evidence = json.loads((root / "evidence.json").read_text())
        evidence["visual_pipeline"]["failed_images"] = {"source.png": "failed-source.png"}
        (root / "evidence.json").write_text(json.dumps(evidence))
        assert state.image_evidence("source.png") is None  # another run cannot supply current frames
        (job / "source.png").write_bytes(b"current-camera")
        assert state.image_evidence("source.png").read_bytes() == b"current-camera"


def check_telemetry_success_is_not_failure():
    with TemporaryDirectory(prefix="rule-workbench-telemetry-") as folder:
        root = Path(folder)
        (root / "result").mkdir()
        state = app.WorkbenchState()
        state.job = {"directory": str(root), "status": "completed"}
        for encode in (lambda reason: reason, lambda reason: {"reason": reason}):
            success = {"tick": 2, "event": encode("success")}
            failure = {"tick": 1, "event": encode("grasp_lost")}
            (root / "result/telemetry-source.jsonl").write_text(json.dumps(success) + "\n")
            (root / "result/telemetry-target.jsonl").write_text("\n".join(map(json.dumps, [failure, success])))
            summary = state.telemetry_summary()
            assert summary["source"]["failure"] is None
            assert summary["source"]["latest"] == success
            assert summary["target"]["failure"] == failure
            assert summary["target"]["latest"] == success


def check_playback_ranges_and_current_job():
    with TemporaryDirectory(prefix="rule-workbench-playback-") as folder:
        root = Path(folder)
        state = app.WorkbenchState()
        state.restore(root)
        state.job = {"directory": str(root / "job-old"), "status": "completed"}
        playback = root / "job-old/result/playback"
        playback.mkdir(parents=True)
        body = b"\x00\x00\x00\x18ftypisom" + bytes(range(64))
        manifest = {"run_id": "test-playback", "clock": "simulation", "files": {}}
        for side in ("source", "target"):
            name = side + "-base_0.mp4"
            (playback / name).write_bytes(body)
            manifest["files"][name] = {"size_bytes": len(body)}
        (playback / "manifest.json").write_text(json.dumps(manifest))
        with serving(state) as request:
            status = request("GET", "/api/playback")
            assert status["cameras"]["base_0"]["ready"]
            assert not status["cameras"]["left_wrist_0"]["ready"]
            url = "/api/playback/source-base_0.mp4"
            assert request("GET", url) == body
            raw, headers = request("GET", url, headers={"Range": "bytes=8-15"}, expected=206, response_headers=True)
            assert raw == body[8:16] and headers["Content-Range"] == f"bytes 8-15/{len(body)}"
            assert headers["Accept-Ranges"] == "bytes"
            assert request("GET", url, headers={"Range": "bytes=-4"}, expected=206) == body[-4:]
            assert request("GET", url, headers={"Range": "bytes=12-"}, expected=206) == body[12:]
            for value in ("bytes=999-", "bytes=9-8", "bytes=0-1,3-4", "bytes=-0", "words=0-4"):
                request("GET", url, headers={"Range": value}, expected=416)
            _, headers = request("HEAD", url, response_headers=True)
            assert headers["Content-Length"] == str(len(body))
            request("GET", "/api/playback/../../state.json", expected=404)
            # A symlink outside the state root is never served.
            outside = root.parent / (root.name + "-outside.mp4")
            try:
                outside.write_bytes(body)
                (playback / "target-base_0.mp4").unlink()
                (playback / "target-base_0.mp4").symlink_to(outside)
                request("GET", "/api/playback/target-base_0.mp4", expected=404)
            finally:
                outside.unlink()
            state.job = {"directory": str(root / "job-new"), "status": "running"}
            assert request("GET", "/api/playback")["status"] == "unavailable"
            request("GET", url, expected=404)
    print("Playback ranges, missing cameras and current-job isolation: PASS")


def check_startup_session_config():
    with TemporaryDirectory(prefix="rule-workbench-startup-") as folder:
        root = Path(folder)
        config = root / "operator-config.json"
        contract_path = root / "operator-contract.json"
        service = {"root": "/mnt/data/feiyu.zhou/runs/detector", "socket": "/mnt/data/feiyu.zhou/runs/detector/s",
                   "owner_token": "original-template-owner", "identity": {"adapter_sha256": "sha256:unchanged"}}
        contract = {"detector_service": deepcopy(service),
                    "filesystem": {"writable_roots": ["/runs/template", service["root"]], "readonly_roots": ["/inputs"]},
                    "provenance": {"config_sha256": {"config.json": "template"}}}
        contract_path.write_text(json.dumps(contract))
        settings = {"detector_service": deepcopy(service), "contract_path": str(contract_path),
                    "remote_run_root": "/runs/template", "workbench_code": "/inputs/current"}
        config.write_text(json.dumps(settings))
        before = (config.read_bytes(), contract_path.read_bytes())
        sessions = []
        for _ in range(2):
            manager = Mock()
            manager.status.return_value = {"status": "ready"}
            state = app.WorkbenchState()
            with patch.object(app, "STATE", state), patch.object(app, "PIPELINE_CONFIG", None), \
                 patch.object(app, "DETECTOR_MANAGER", None), patch.object(app, "INITIALIZATION_ERROR", None), \
                 patch.object(app, "ThreadingHTTPServer") as server, patch("detector_service.DetectorService", return_value=manager), \
                 patch("signal.signal"), patch("sys.argv", ["app", "--pipeline-config", str(config), "--state-dir", str(root)]):
                app.main()
                manager.start.assert_called_once_with(app.PIPELINE_CONFIG, root.resolve())
                manager.close.assert_called_once()
                server.return_value.serve_forever.assert_called_once()
                derived = json.loads(app.PIPELINE_CONFIG.read_text())
                grant = json.loads(Path(derived["contract_path"]).read_text())
                sessions.append(derived["detector_service"])
                assert derived["detector_service"] == grant["detector_service"]
                assert grant["filesystem"]["writable_roots"] == ["/runs/template", derived["detector_service"]["root"]]
                assert derived["detector_service"]["identity"] == service["identity"]
                assert len(derived["detector_service"]["socket"].encode()) < 104
                assert grant["provenance"]["config_sha256"]["config.json"] == hashlib.sha256(app.PIPELINE_CONFIG.read_bytes()).hexdigest()
                with serving(state) as request, patch.object(app, "Thread"):
                    job = request("POST", "/api/pipeline/run", {}, expected=202)
                    job_config = json.loads((Path(job["directory"]) / "config.json").read_text())
                    assert job_config["detector_service"] == derived["detector_service"]
                # Avoid restoring the synthetic running job as needs-review next loop.
                state.job = None
                state.persist()
        assert sessions[0]["root"] != sessions[1]["root"]
        assert sessions[0]["owner_token"] != sessions[1]["owner_token"] != service["owner_token"]
        assert (config.read_bytes(), contract_path.read_bytes()) == before
        legacy = root / "legacy.json"
        legacy.write_text('{}')
        assert app.startup_config(legacy, root) == legacy
        settings["detector_service"]["root"] = "/" + "a" * 96
        settings["detector_service"]["socket"] = settings["detector_service"]["root"] + "/s"
        contract["detector_service"] = deepcopy(settings["detector_service"])
        contract["filesystem"]["writable_roots"] = [settings["detector_service"]["root"]]
        config.write_text(json.dumps(settings)); contract_path.write_text(json.dumps(contract))
        try:
            app.startup_config(config, root)
        except ValueError as error:
            assert "104" in str(error)
        else:
            raise AssertionError("oversized detector socket accepted")
    print("Unique app detector session, unchanged template, socket bound and job binding: PASS")


if __name__ == "__main__":
    main()
    check_failed_visual_run_is_current()
    check_telemetry_success_is_not_failure()
    check_playback_ranges_and_current_job()
    check_startup_session_config()
    print("Failed visual run precedence and read-only plan guard: PASS")
