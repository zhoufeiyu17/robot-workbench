"""Standalone, dependency-free Rule Based data collection workbench."""

from __future__ import annotations

import json
import os
import hashlib
import secrets
import subprocess
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
import sys
import threading
import time
from threading import Thread
from datetime import datetime, timezone
from uuid import uuid4
from urllib.parse import parse_qs, urlsplit

from contract import demo_state, validate_boundary, validate_command, validate_physical14
from integration import DualGpuPipeline, ObservationWindow, _digest, freeze_receipt, validate_profile
from live_preview import read_frame, camera_frame_path
from runtime_progress import snapshot as runtime_snapshot


ROOT = Path(__file__).parent
PLAYBACK_CAMERAS = ("base_0", "left_wrist_0", "right_wrist_0")
PLAYBACK_FILES = {f"{side}-{camera}.mp4" for side in ("source", "target") for camera in PLAYBACK_CAMERAS}


class WorkbenchState:
    def __init__(self):
        self.directory = None
        self.profile = None
        self.window = None
        self.scene = None
        self.receipt = None
        self.last_action = demo_state()
        self.pipeline = DualGpuPipeline()
        self.job = None

    def playback_root(self):
        if not self.directory:
            return None
        if self.job:
            if not self.job.get("directory"):
                return None
            root = Path(self.job["directory"]) / "result/playback"
        else:
            root = self.directory / "raw/playback"
        root = root.resolve()
        return root if root.is_relative_to(self.directory.resolve()) else None

    def playback_manifest(self):
        root = self.playback_root()
        if root is None:
            return {}
        path = (root / "manifest.json").resolve()
        if not path.is_relative_to(root):
            return {}
        try:
            manifest = json.loads(path.read_text())
            return manifest if isinstance(manifest, dict) and isinstance(manifest.get("files"), dict) else {}
        except (OSError, ValueError):
            return {}

    def playback_path(self, name):
        root = self.playback_root()
        manifest = self.playback_manifest()
        if root is None or name not in PLAYBACK_FILES or not manifest.get("run_id") or manifest.get("clock") != "simulation":
            return None
        path = (root / name).resolve()
        if not path.is_relative_to(self.directory.resolve()) or not path.is_file():
            return None
        entry = manifest["files"].get(name, {})
        if not isinstance(entry, dict) or entry.get("size_bytes") != path.stat().st_size:
            return None
        with path.open("rb") as stream:
            if stream.read(8)[4:] != b"ftyp":
                return None
        return path

    def playback_status(self):
        cameras = {}
        for camera in PLAYBACK_CAMERAS:
            sides = {}
            for side in ("source", "target"):
                name = f"{side}-{camera}.mp4"
                path = self.playback_path(name)
                sides[side] = {"available": bool(path), "url": "/api/playback/" + name if path else None,
                               "revision": f"{path.stat().st_mtime_ns}-{path.stat().st_size}" if path else None}
            cameras[camera] = {**sides, "ready": all(item["available"] for item in sides.values())}
        manifest = self.playback_manifest()
        return {"clock": "simulation", "cameras": cameras, "run_id": manifest.get("run_id"),
                "timeline_digest": manifest.get("timeline_digest"),
                "status": "ready" if any(item["ready"] for item in cameras.values()) else "unavailable"}

    def persist(self):
        if self.directory:
            data = {key: getattr(self, key) for key in ("profile", "scene", "receipt", "last_action", "job")}
            data["frames"] = self.window.frames if self.window else []
            data["window_active"] = self.window is not None
            data["reobserve_events"] = self.window.reobserve_events if self.window else []
            data["cup_motion_threshold_m"] = self.window.cup_motion_threshold_m if self.window else None
            data["cup_position_jitter_m"] = self.window.cup_position_jitter_m if self.window else None
            data["cup_motion_threshold_source"] = self.window.cup_motion_threshold_source if self.window else "unresolved"
            data["pipeline"] = {"source": self.pipeline.source, "plan": self.pipeline.plan,
                                "executions": self.pipeline.executions}
            temp = self.directory / "state.json.partial"
            temp.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False))
            temp.replace(self.directory / "state.json")

    def restore(self, directory):
        self.directory = directory = Path(directory).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "state.json"
        if not path.exists():
            return
        data = json.loads(path.read_text())
        self.job = data.get("job")
        if self.job and self.job.get("status") == "running":
            self.job = {**self.job, "status": "needs-review", "error": "服务重启；先核验远端运行终态，禁止自动重启采集"}
        self.profile = validate_profile(data["profile"]) if data.get("profile") else None
        active = data.get("window_active", not data.get("pipeline", {}).get("source"))
        self.window = (ObservationWindow(
            self.profile, data.get("frames", []),
            cup_motion_threshold_m=data.get("cup_motion_threshold_m"),
            cup_position_jitter_m=data.get("cup_position_jitter_m"),
            cup_motion_threshold_source=data.get("cup_motion_threshold_source", "unresolved"),
            reobserve_events=data.get("reobserve_events", [])) if self.profile and active else None)
        self.scene, self.receipt = data.get("scene"), data.get("receipt")
        self.last_action = validate_physical14(data["last_action"])
        p = data.get("pipeline", {})
        try:
            if p.get("source"):
                self.pipeline.ingest_source(p["source"])
            if p.get("plan"):
                self.pipeline.accept_plan(p["plan"])
            for value in p.get("executions", {}).values():
                self.pipeline.record_execution(value)
        except ValueError:
            # Historical receipts can reference a retired lane allocation; keep
            # them available through evidence without preventing the UI from booting.
            self.pipeline = DualGpuPipeline()

    def telemetry_summary(self):
        """Expose a bounded, honest tail of replay telemetry to the workbench."""
        result = {"status": "unavailable", "source": {"rows": []}, "target": {"rows": []}}
        if not self.job or not self.job.get("directory"):
            return result
        root = Path(self.job["directory"]) / "result"
        any_rows = False
        for label in ("source", "target"):
            path = root / ("telemetry-" + label + ".jsonl")
            rows = []
            if path.is_file():
                try:
                    for line in path.read_text().splitlines():
                        try:
                            item = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        if isinstance(item, dict) and isinstance(item.get("tick"), int):
                            rows.append(item)
                except OSError:
                    rows = []
            any_rows |= bool(rows)
            tail = rows[-180:]
            failures = [item for item in rows if item.get("event") and (
                item["event"].get("reason") if isinstance(item["event"], dict) else item["event"]
            ) != "success"]
            result[label] = {"rows": tail, "row_count": len(rows),
                             "latest": tail[-1] if tail else None,
                             "failure": failures[-1] if failures else None}
        if any_rows:
            result["status"] = "running" if self.job.get("status") == "running" else "complete"
        return result

    def live_preview(self, side, camera="base_0"):
        if side not in {"source", "target"} or not self.job or not self.job.get("directory"):
            return None
        path = camera_frame_path(Path(self.job["directory"]) / "result" / ("replay-" + side), camera)
        frame = read_frame(path)
        if frame is None:
            return None
        metadata = dict(frame.metadata)
        metadata["age_s"] = max(0.0, (time.time_ns() - int(metadata.get("published_at_ns", time.time_ns()))) / 1e9)
        return frame, metadata

    def current_job_artifact(self, name):
        if not self.job or not self.job.get("directory"):
            return None
        root = Path(self.job["directory"]) / "result"
        for path in (root / name, root / ".artifacts" / name):
            try:
                raw = path.read_text()
                value = json.JSONDecoder().raw_decode(raw[raw.find("{"):])[0]
                if isinstance(value, dict):
                    return value
            except (OSError, ValueError, TypeError):
                pass
        return None

    def unpromoted_job(self):
        return bool(self.job and self.job.get("status") != "completed")

    def summary(self):
        result = {
            "state": self.last_action,
            "adapters": {
                "calibration": {"status": "ready" if self.profile else "awaiting-profile",
                                 "detail": "versioned calibration profile" if self.profile else "需要相机/关节/夹爪 profile"},
                "online_detection": {"status": "ready" if self.scene else "awaiting-5s-window",
                                      "detail": self.window.status() if self.window else "等待 FoundationPose snapshot"},
                "simulation": {"status": self.receipt["state"] if self.receipt else "awaiting-scene",
                                "detail": "trajectory receipt" if self.receipt else "等待仿真 rollout"},
                "control": {"status": "dry-run-only", "detail": "真机 adapter 尚未连接；不会发送控制命令"},
            },
            "calibration_profile": None if not self.profile else {
                "profile_id": self.profile["profile_id"], "schema_version": self.profile["schema_version"],
                "digest": _digest(self.profile),
            },
            "scene": self.scene,
            "trajectory": None if not self.receipt else {
                "state": self.receipt["state"], "receipt_digest": self.receipt["receipt_digest"],
                "tick_count": len(self.receipt["ticks"]),
            },
            "pipeline": self.pipeline.summary(),
            "job": self.job,
            "telemetry": self.telemetry_summary(),
            "live_preview": {side: (self.live_preview(side)[1] if self.live_preview(side) else None)
                             for side in ("source", "target")},
            "pipeline_configured": PIPELINE_CONFIG is not None,
            "initialization": initialization_status(),
        }
        if self.pipeline.source:
            result["pipeline"]["source_run_id"] = self.pipeline.source["run_id"]
        if self.pipeline.plan:
            result["pipeline"]["run_id"] = self.pipeline.plan["run_id"]
            result["pipeline"]["plan_run_id"] = self.pipeline.plan["run_id"]
            result["pipeline"]["stages"] = self.pipeline.plan.get("stages", [])
            result["adapters"]["simulation"] = {"status": "planned", "detail": "Isaac native rollout timeline"}
        traces = self.pipeline.executions or {}
        if {"simulated-real", "dsw-scene"} <= traces.keys():
            pairs = list(zip(traces["simulated-real"]["measured_states"], traces["dsw-scene"]["measured_states"]))
            result["pipeline"]["comparison"] = {
                "max_joint_error_rad": max(abs(x-y) for a,b in pairs for key in ("left_arm_joints", "right_arm_joints") for x,y in zip(a[key],b[key])),
                "max_gripper_error_mm": max(abs(a[key]-b[key]) for a,b in pairs for key in ("left_gripper", "right_gripper")),
                "scope": "measured difference between two PhysX lanes; not hardware tracking error"}
        result["evidence"] = {}
        if self.directory and (self.directory / "evidence.json").exists():
            result["evidence"] = json.loads((self.directory / "evidence.json").read_text())
        # A failed visual run is newer than any previously promoted GT replay.
        # Keep the old receipt available for audit, but expose the failed run as
        # the only current run so the UI cannot present stale success as current.
        visual = result["evidence"].get("visual_pipeline") if isinstance(result["evidence"], dict) else None
        if isinstance(visual, dict) and visual.get("status") == "failed":
            plan = self.latest_visual_plan()
            result["current_run"] = {
                "run_id": visual.get("run_id"), "status": "failed", "promoted": False,
                "historical_pipeline_preserved": bool(self.pipeline.source or self.pipeline.plan),
                "detection": visual.get("detection"), "planning": visual.get("planning"),
                "execution": visual.get("execution"),
                "plan_available": plan is not None,
                "plan_source": "failed-job-artifact" if plan else None,
                "plan": ({"run_id": plan.get("run_id"), "timeline_digest": plan.get("timeline_digest"),
                           "tick_count": len(plan.get("ticks", [])), "validated": False}
                          if plan else None),
            }
        else:
            result["current_run"] = {"run_id": None, "status": "promoted" if self.pipeline.plan else "waiting",
                                      "promoted": bool(self.pipeline.plan), "plan_available": bool(self.pipeline.plan),
                                      "plan_source": "promoted-receipt" if self.pipeline.plan else None}
        if self.job:
            progress = Path(self.job["directory"]) / "result/progress.json"
            if progress.is_file():
                result["job"] = {**self.job, "progress": json.loads(progress.read_text()),
                                 "runtime": runtime_snapshot(progress.parent, self.job["status"], time.time())}
        if self.unpromoted_job():
            # A new job owns every current result panel; promoted history is not its fallback.
            current = DualGpuPipeline()
            for name, ingest in (("source.json", current.ingest_source), ("plan.json", current.accept_plan),
                                 ("execution-source.json", current.record_execution),
                                 ("execution-target.json", current.record_execution)):
                artifact = self.current_job_artifact(name)
                if artifact:
                    try:
                        ingest(artifact)
                    except (ValueError, KeyError, TypeError):
                        pass
            source, plan = current.source, self.latest_visual_plan()
            result["pipeline"] = current.summary()
            if source:
                result["pipeline"]["source_run_id"] = source["run_id"]
            if plan:
                result["pipeline"].update(plan_run_id=plan.get("run_id"), timeline_digest=plan.get("timeline_digest"),
                                          tick_count=len(plan["ticks"]))
            result["scene"] = source["scene"] if source else None
            result["calibration_profile"] = ({"profile_id": source["calibration_profile"]["profile_id"],
                "digest": _digest(source["calibration_profile"])} if source else None)
            result["trajectory"] = None
            result["state"] = None
            for adapter in ("calibration", "online_detection", "simulation"):
                result["adapters"][adapter] = {"status": "unverified", "detail": "仅显示本轮已取得的证据"}
            result["current_run"] = {"run_id": Path(self.job["directory"]).name, "status": self.job["status"],
                "promoted": False, "plan_available": bool(plan), "plan_source": "current-job-artifact" if plan else None,
                "plan": ({"run_id": plan.get("run_id"), "timeline_digest": plan.get("timeline_digest"),
                          "tick_count": len(plan["ticks"]), "validated": False} if plan else None)}
            result["evidence"] = {"status": self.job["status"], "phase": "本轮采集", "message": self.job.get("error", "等待本轮完整回执")}
        return result

    def latest_visual_plan(self):
        """Read a failed job's exported plan without promoting it into pipeline state."""
        if not self.job or not self.job.get("directory"):
            return None
        value = self.current_job_artifact("plan.json")
        return value if value and isinstance(value.get("ticks"), list) and value["ticks"] else None

    def image_evidence(self, name):
        if self.unpromoted_job():
            root = Path(self.job["directory"]) / "result"
            for path in (root / name, root / ".artifacts" / name):
                if path.is_file():
                    return path
            return None
        source, plan = self.pipeline.source, self.pipeline.plan
        if not self.directory:
            return None
        manifest = self.directory / "evidence.json"
        if not manifest.is_file():
            return None
        evidence = json.loads(manifest.read_text())
        # A failed visual run supersedes the promoted receipt. Historical
        # frames must not be rendered as if they belonged to that run. A
        # failed run may still publish explicitly bound camera frames.
        visual = evidence.get("visual_pipeline")
        if isinstance(visual, dict) and visual.get("status") == "failed":
            failed_reference = visual.get("failed_images", {}).get(name)
            if not isinstance(failed_reference, str):
                return None
            path = (self.directory / failed_reference).resolve()
            return path if path.is_relative_to(self.directory.resolve()) and path.is_file() else None
        if not source:
            return None
        reference = evidence.get("images", {}).get(name, {})
        if (reference.get("source_run_id") != source["run_id"]
                or reference.get("source_sequence") != source["sequence"]
                or reference.get("scene_digest") != source["scene"]["scene_digest"]
                or (name == "target.png" and (not plan or reference.get("plan_run_id") != plan["run_id"]))):
            return None
        path = (self.directory / reference.get("path", name)).resolve()
        return path if path.is_relative_to(self.directory.resolve()) and path.is_file() else None


STATE = WorkbenchState()
LOCK = threading.RLock()
PIPELINE_CONFIG = None
DETECTOR_MANAGER = None
PLANNER_MANAGER = None
INITIALIZATION_ERROR = None
REAL_BRIDGE = None
REAL_JOB = None
REAL_STATE_DIR = None
PLAYGROUND = None
WORKBENCH_MODE = 'dual_dsw'
CSRF_TOKEN = secrets.token_urlsafe(32)


def playground_service():
    global PLAYGROUND
    if PLAYGROUND is None and REAL_BRIDGE is not None and REAL_STATE_DIR is not None:
        from playground import Playground
        PLAYGROUND = Playground(REAL_STATE_DIR / 'playground', REAL_BRIDGE, initialization_status,
                                lambda: bool(STATE.job and STATE.job.get('status') in ('running', 'needs-review'))
                                or bool(REAL_JOB and REAL_JOB.get('status') in ('running', 'needs-review')),
                                recover_planner=recover_playground_planner)
    return PLAYGROUND


def recover_playground_planner(*,progress=None):
    if PLANNER_MANAGER is not None:
        PLANNER_MANAGER.recover(progress=progress)


def persist_real_job():
    if REAL_STATE_DIR:
        REAL_STATE_DIR.mkdir(parents=True, exist_ok=True)
        value = {'job': REAL_JOB, 'artifacts': {key: REAL_BRIDGE.config.get(key) for key in
                 ('source_path', 'plan_path', 'hardware_binding', 'artifact_dir')}}
        path = REAL_STATE_DIR / 'state.json'
        temporary = path.with_suffix('.json.partial')
        temporary.write_text(json.dumps(value, ensure_ascii=False))
        temporary.replace(path)


def publish_real_binding(root):
    """Publish only pipeline-validated artifacts to the startup-configured owner file."""
    configured = REAL_BRIDGE.config.get('trusted_binding_path')
    if configured is None:
        return  # Legacy/fake deployments retain their existing artifact handling.
    from real_contract import validate_real_plan
    target = Path(configured)
    if not target.is_absolute() or target.is_symlink() or not target.is_file():
        raise ValueError('trusted binding must be an existing absolute regular file')
    source = json.loads((root / 'source.json').read_text())
    plan = json.loads((root / 'plan.json').read_text())
    data = (root / 'hardware-binding.json').read_bytes()
    binding = json.loads(data)
    validate_real_plan(plan, source, binding)
    previous = target.read_bytes()
    old = json.loads(previous)
    if any(old.get(key) != binding.get(key) for key in ('robot_id', 'camera_serials')):
        raise ValueError('new binding differs from trusted robot/camera identity')
    owner = REAL_BRIDGE.owner()  # GET /state only; never preview, acquire or command.
    if (owner.get('automatic_enabled') is not False or owner.get('acquisition_pending')
            or owner.get('stationary') is not True
            or owner.get('state') not in ('READY', 'STOPPED_DAMPING_HOLD', 'MANUAL_TAKEOVER')
            or (owner.get('motion') or {}).get('status') in ('VERIFYING', 'RUNNING', 'SETTLING', 'AWAITING_CAPTURE')):
        raise ValueError('controller must be stationary and not executing before binding publication')
    (root / 'hardware-binding.previous.json').write_bytes(previous)
    receipt = dict(path=str(target), previous_sha256=hashlib.sha256(previous).hexdigest(),
                   published_sha256=hashlib.sha256(data).hexdigest(), source_digest=binding['source_digest'])
    temporary = target.with_name(target.name + '.' + uuid4().hex + '.tmp')
    try:
        with temporary.open('xb') as stream:
            os.chmod(temporary, 0o600)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if target.read_bytes() != previous:
            raise ValueError('trusted binding changed during preparation')
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    (root / 'hardware-binding-publication.json').write_text(json.dumps(receipt, indent=2))


def run_real_preparation(config, output, preparation):
    global REAL_JOB
    try:
        from real_pipeline import run
        if 'acquisition' in preparation:
            if REAL_BRIDGE.config.get('hardware_access_enabled') is not True:
                raise ValueError('自动真实观测需要已交接的相机访问窗口')
            from real_acquisition import prepare_inputs
            with LOCK:
                REAL_JOB['phase'] = 'physical-observation'
                persist_real_job()
            preparation = prepare_inputs({**preparation, 'pipeline_config': str(config)}, output,
                read_snapshot=lambda: REAL_BRIDGE.camera('base_0', 'snapshot.zip')[0], read_owner=REAL_BRIDGE.owner)
            config = Path(preparation['pipeline_config'])
            with LOCK:
                REAL_JOB['phase'] = 'physical-preparation'
                persist_real_job()
        result = run(config, output / 'result', capture_dir=preparation['capture_dir'],
                     physical_binding=preparation['physical_binding'])
        if result['status'] != 'prepared':
            raise ValueError(result.get('error', '真实场景规划失败'))
        with LOCK:
            root = output / 'result'
            publish_real_binding(root)
            REAL_BRIDGE.config.update(source_path=str(root / 'source.json'), plan_path=str(root / 'plan.json'),
                                      hardware_binding=str(root / 'hardware-binding.json'), artifact_dir=str(root))
            REAL_JOB = {**REAL_JOB, 'status': 'prepared', 'result': str(root)}
            persist_real_job()
    except Exception as error:
        with LOCK:
            REAL_JOB = {**REAL_JOB, 'status': 'failed', 'error': str(error)}
            persist_real_job()


def monitor_real_execution(plan, previous_recording=None, *, sleep=time.sleep, clock=time.monotonic):
    """Observe the confirmed run, then finalize only its synchronized raw evidence."""
    global REAL_JOB
    from real_workflow import finalize_collection
    deadline = clock() + float(plan['duration_s']) + 60
    try:
        while clock() < deadline:
            owner = REAL_BRIDGE.owner()
            frozen = owner.get('frozen') or {}
            recording = owner.get('recording') or {}
            if frozen and frozen.get('timeline_digest') != plan['timeline_digest']:
                raise ValueError('真机当前时间线改变；不能将另一轮终态记入本轮')
            if recording.get('done') and recording.get('directory') != previous_recording:
                with LOCK:
                    REAL_JOB['execution_finished_unix_s'] = time.time()
                if not (REAL_BRIDGE.config.get('raw_source_dir') or REAL_BRIDGE.config.get('local_recording_root')):
                    with LOCK:
                        REAL_JOB.update(status='awaiting-artifacts', phase='raw-transfer',
                            error='真机 raw 已封存；等待本轮 raw 运输与 raw_source_dir 配置后生成回放')
                        persist_real_job()
                    return
                with LOCK:
                    REAL_JOB['phase'] = 'export-playback'
                result = finalize_collection(REAL_BRIDGE.config, recording)
                with LOCK:
                    REAL_JOB.update(status='completed' if result['status'] == 'ready' else 'failed',
                                    phase='complete', finalization=result)
                    if result.get('error'):
                        REAL_JOB['error'] = result['error']
                    persist_real_job()
                return
            if not owner.get('connected'):
                raise ValueError('真机执行期间控制服务断开；终态需要现场核验')
            sleep(.25)
        raise ValueError('真机执行/录制终态等待超时；不自动重试或续跑')
    except Exception as error:
        with LOCK:
            REAL_JOB.update(status='needs-review', error=str(error), execution_finished_unix_s=time.time())
            persist_real_job()


def watch_confirmed_execution(plan, previous_recording=None):
    global REAL_JOB
    if REAL_JOB and REAL_JOB.get('status') in ('running', 'needs-review'):
        raise ValueError('已有真实采集运行或待核验终态')
    if not REAL_JOB:
        if REAL_STATE_DIR is None:
            raise ValueError('真机执行需要持久化 state-dir')
        output = REAL_STATE_DIR / 'jobs' / ('execution-' + uuid4().hex)
        output.mkdir(parents=True, exist_ok=False)
        REAL_JOB = {'directory': str(output), 'mode': 'real'}
    REAL_JOB.update(status='running', phase='physical-execution',
                    execution_started_unix_s=time.time(), execution_estimate_s=plan['duration_s'])
    REAL_JOB.pop('error', None)
    REAL_JOB.pop('execution_finished_unix_s', None)
    persist_real_job()
    Thread(target=monitor_real_execution, args=(plan, previous_recording), daemon=True).start()


def start_real_preparation():
    """Reuse frozen deployment/contract, with a fresh non-overwriting output root."""
    global REAL_JOB
    preparation = REAL_BRIDGE.config.get('preparation') or {}
    if set(preparation) not in ({'pipeline_config', 'capture_dir', 'physical_binding'}, {'pipeline_config', 'acquisition'}) or REAL_STATE_DIR is None:
        raise ValueError('真实观测规划配置不完整：需要冻结配置、RGB-D capture 与已验收标定绑定')
    if REAL_JOB and REAL_JOB.get('status') in ('running', 'needs-review'):
        raise ValueError('真实观测规划运行中或等待核验')
    if 'acquisition' in preparation:
        if REAL_BRIDGE.config.get('hardware_access_enabled') is not True:
            raise ValueError('自动真实观测需要已交接的相机访问窗口')
        from real_acquisition import validate_config
        validate_config(preparation)
    base = json.loads(Path(preparation['pipeline_config']).read_text())
    if base.get('detector_service') and initialization_status().get('status') != 'ready':
        raise ValueError('检测器尚未就绪；等待工作台初始化完成后重试')
    contract = json.loads(Path(base['contract_path']).read_text())
    output = REAL_STATE_DIR / 'jobs' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    previous = base['remote_run_root']
    base['remote_run_root'] = previous.rstrip('/') + '-physical-' + uuid4().hex
    contract['filesystem']['writable_roots'] = [base['remote_run_root'] if p == previous else p
                                                for p in contract['filesystem']['writable_roots']]
    contract.setdefault('identity', {})['experiment_id'] = output.name
    base['contract_path'] = str(output / 'contract.json')
    config = output / 'config.json'
    config.write_text(json.dumps(base, ensure_ascii=False, indent=2))
    if 'config_sha256' in contract.get('provenance', {}):
        contract['provenance']['config_sha256'] = {'config.json': hashlib.sha256(config.read_bytes()).hexdigest()}
    (output / 'contract.json').write_text(json.dumps(contract, ensure_ascii=False, indent=2))
    from real_pipeline import run
    # Validate paths and pin coverage before any remote operation or running state.
    if 'acquisition' not in preparation:
        run(config, output / 'result', capture_dir=preparation['capture_dir'],
            physical_binding=preparation['physical_binding'], dry_run=True)
    REAL_JOB = {'status': 'running', 'mode': 'real', 'directory': str(output)}
    # A new run owns every current panel. Never show the previous plan on failure.
    for key in ('source_path', 'plan_path', 'hardware_binding'):
        REAL_BRIDGE.config.pop(key, None)
    REAL_BRIDGE.config['artifact_dir'] = str(output / 'result')
    persist_real_job()
    Thread(target=run_real_preparation, args=(config, output, preparation), daemon=True).start()
    return REAL_JOB


def initialization_status():
    if PIPELINE_CONFIG is None:
        return {"status": "disabled", "stage": "unconfigured", "progress_percent": None,
                "message": "未配置采集环境；可查看历史回放", "elapsed_s": 0, "eta_s": None}
    if INITIALIZATION_ERROR:
        return {"status": "failed", "stage": "startup_error", "progress_percent": None,
                "message": "检测器初始化失败", "error": INITIALIZATION_ERROR, "elapsed_s": 0, "eta_s": None}
    if DETECTOR_MANAGER is None:
        return {"status": "starting", "stage": "queued", "progress_percent": 0,
                "message": "等待启动检测器初始化", "elapsed_s": 0, "eta_s": None}
    try:
        result=DETECTOR_MANAGER.status()
        if PLANNER_MANAGER is not None:
            planner=PLANNER_MANAGER.status();result['planner']=planner
            if planner['status'] in ('failed','stopped'):
                result.update(status='failed',error=planner.get('error') or '规划进程已停止，需要恢复规划环境',message='规划环境尚未就绪')
            elif planner['status']=='starting' and result.get('status')=='ready':
                result.update(status='starting',message='正在准备规划环境')
        return result
    except Exception as error:
        return {"status": "failed", "stage": "status_error", "progress_percent": None,
                "message": "读取初始化状态失败", "error": str(error), "elapsed_s": 0, "eta_s": None}


def startup_config(template, directory, *, shared_service=None):
    """Freeze one detector ownership scope per app lifetime, never retry a service."""
    base = json.loads(Path(template).read_text())
    if base.get("detector_service") is None:
        return Path(template)
    contract = json.loads(Path(base["contract_path"]).read_text())
    service = base["detector_service"]
    if service != contract.get("detector_service"):
        raise ValueError("检测器配置与启动合同不匹配")
    old_root = service["root"]
    if (old_root not in contract["filesystem"]["writable_roots"]
            or PurePosixPath(service["socket"]).parent != PurePosixPath(old_root)):
        raise ValueError("检测器启动目录或 socket 不在冻结合同范围内")
    session = uuid4().hex
    new_root = old_root.rstrip("/") + "-" + session[:8]
    socket = new_root + "/" + PurePosixPath(service["socket"]).name
    if len(socket.encode()) >= 104:
        raise ValueError("派生检测器 socket 路径过长（必须少于 104 字节）；请缩短模板 service root")
    if shared_service is not None:
        ownership = {'root', 'socket', 'owner_token'}
        if ({k:v for k,v in service.items() if k not in ownership} !=
                {k:v for k,v in shared_service.items() if k not in ownership}):
            raise ValueError('双 DSW 与真机检测器身份、资源或路径范围不一致')
        service = dict(shared_service)
        new_root = service['root']
    else:
        service.update(root=new_root, socket=socket, owner_token=uuid4().hex)
    base['detector_service'] = service
    contract["detector_service"] = dict(service)
    contract["filesystem"]["writable_roots"] = [new_root if path == old_root else path
                                               for path in contract["filesystem"]["writable_roots"]]
    destination = Path(directory).resolve() / "startups" / session
    destination.mkdir(parents=True, mode=0o700, exist_ok=False)
    config = destination / "config.json"
    contract_path = destination / "contract.json"
    base["contract_path"] = str(contract_path)
    config.write_text(json.dumps(base, ensure_ascii=False, indent=2))
    config.chmod(0o600)
    if "config_sha256" in contract.get("provenance", {}):
        contract["provenance"]["config_sha256"] = {"config.json": hashlib.sha256(config.read_bytes()).hexdigest()}
    contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2))
    contract_path.chmod(0o600)
    return config


def shared_startup_config(template, directory, real_config=None):
    """Freeze one service identity while preserving each mode's pipeline inputs."""
    preparation = (real_config or {}).get('preparation')
    real_template = preparation.get('pipeline_config') if preparation else None
    template = template or real_template
    if template is None:
        return None
    config = startup_config(template, directory)
    from planner_service import startup_config as planner_startup
    config = planner_startup(config,directory)
    candidates=[item for item in (preparation,(real_config or {}).get('playground'))
                if item and item.get('pipeline_config')]
    for candidate in candidates:
        candidate_template=candidate['pipeline_config']
        if Path(candidate_template).resolve() == Path(template).resolve():
            candidate['pipeline_config'] = str(config)
        else:
            service = json.loads(config.read_text()).get('detector_service')
            real_service = json.loads(Path(candidate_template).read_text()).get('detector_service')
            if bool(service) != bool(real_service):
                raise ValueError('双 DSW 与真机必须共享同一检测器启动方式')
            rebased=startup_config(candidate_template, directory, shared_service=service)
            candidate['pipeline_config']=str(planner_startup(rebased,directory,shared_config=config))
    return config


def run_pipeline_job(config, output):
    """Run the installed orchestrator, then atomically promote its validated receipts."""
    try:
        with (output / "controller.log").open("x") as log:
            result = subprocess.run([sys.executable, str(ROOT / "run_pipeline.py"), "--config", str(config),
                                     "--output", str(output / "result")], stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            final_path = output / "result/final.json"
            detail = json.loads(final_path.read_text()).get("error") if final_path.is_file() else None
            raise ValueError(f"采集命令退出 {result.returncode}：{detail or '请查看 controller.log'}；日志 {output / 'controller.log'}")
        artifacts = output / "result"
        if STATE.job.get("mode") == "diagnostic":
            final = json.loads((artifacts / "final.json").read_text())
            with LOCK:
                STATE.job = {**STATE.job, "status": final.get("status", "failed"), "result": str(artifacts),
                             "diagnostic": final.get("diagnostic", {}), "promotion": "forbidden"}
                STATE.persist()
            return
        candidate = DualGpuPipeline()
        source = json.loads((artifacts / "source.json").read_text())
        if source.get("adapter") != "foundationpose":
            raise ValueError("正式视觉 pipeline 必须使用 FoundationPose")
        candidate.ingest_source(source)
        candidate.accept_plan(json.loads((artifacts / "plan.json").read_text()))
        for side, lane in (("source", "simulated-real"), ("target", "dsw-scene")):
            receipt = json.loads((artifacts / f"execution-{side}.json").read_text())
            if receipt.get("side") != lane:
                raise ValueError("执行回执的文件与实例身份不一致")
            candidate.record_execution(receipt)
        final = json.loads((artifacts / "final.json").read_text())
        if final.get("status") not in {"completed", "replay-complete"}:
            raise ValueError("pipeline 未提供完整终态")
        with LOCK:
            reference = {"source_run_id": source["run_id"], "source_sequence": source["sequence"],
                         "scene_digest": source["scene"]["scene_digest"], "plan_run_id": candidate.plan["run_id"]}
            evidence = {"phase": "双侧执行完成", "status": "completed",
                        "message": "FoundationPose 场景传递、原生规划、双侧执行与录制完成；实时同步及录制完整性见各自验收。",
                        "images": {name: {**reference, "path": str((artifacts / name).relative_to(STATE.directory))}
                                   for name in ("source.png", "target.png")}}
            temp = STATE.directory / "evidence.json.partial"
            temp.write_text(json.dumps(evidence, ensure_ascii=False))
            temp.replace(STATE.directory / "evidence.json")
            STATE.pipeline = candidate
            STATE.profile, STATE.scene = source["calibration_profile"], source["scene"]
            STATE.window = STATE.receipt = None
            STATE.last_action = candidate.executions["simulated-real"]["measured_states"][-1]
            STATE.job = {**STATE.job, "status": "completed", "result": str(artifacts)}
            STATE.persist()
    except Exception as error:
        with LOCK:
            STATE.job = {**STATE.job, "status": "failed", "error": str(error)}
            STATE.persist()


class Handler(BaseHTTPRequestHandler):
    server_version = "RuleBasedWorkbench/0.1"

    def _send(self, status: int, body: object, content_type: str = "application/json") -> None:
        raw = body if isinstance(body, bytes) else (body.encode() if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode())
        self.send_response(status)
        self.send_header("Content-Type", content_type if isinstance(body, bytes) else f"{content_type}; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _json(self) -> object:
        length = int(self.headers.get("Content-Length", "0"))
        if not 0 <= length <= 16 * 1024 * 1024:
            raise ValueError("request must be at most 16 MiB")
        return json.loads(self.rfile.read(length) or b"{}")

    def request_mode(self):
        values = parse_qs(urlsplit(self.path).query).get('mode', [WORKBENCH_MODE])
        if len(values) != 1 or values[0] not in ('dual_dsw', 'real'):
            raise ValueError('unknown workbench mode')
        return values[0]

    def request_camera(self):
        values = parse_qs(urlsplit(self.path).query).get('camera', ['base_0'])
        if len(values) != 1 or values[0] not in PLAYBACK_CAMERAS:
            raise ValueError('unknown camera')
        return values[0]

    def real_get(self, route):
        """Shared API, exclusively backed by physical acquisition evidence."""
        if REAL_BRIDGE is None:
            self._send(HTTPStatus.SERVICE_UNAVAILABLE, {'error': '真机独立服务尚未配置'})
            return
        if route == '/api/playback':
            self._send(HTTPStatus.OK, REAL_BRIDGE.playback_status())
            return
        if route == '/api/pipeline/plan':
            try:
                _, plan = REAL_BRIDGE.inputs()
            except (OSError, ValueError, KeyError) as error:
                plan = {'status': 'unavailable', 'validated': False, 'error': str(error)}
            self._send(HTTPStatus.OK, plan)
            return
        if route.startswith('/api/live-preview/'):
            name = route.rsplit('/', 1)[-1]
            side, suffix = name.rsplit('.', 1)
            live = REAL_BRIDGE.live_preview(side, self.request_camera())
            if live and suffix == 'jpg':
                self._send(HTTPStatus.OK, live[0].payload, 'image/jpeg')
            else:
                self._send(HTTPStatus.OK if live and suffix == 'json' else HTTPStatus.NOT_FOUND,
                           live[1] if live else {'status': 'unavailable'})
            return
        if route.startswith('/api/evidence/'):
            path = REAL_BRIDGE.image_evidence(route.rsplit('/', 1)[-1])
            if path:
                self._send(HTTPStatus.OK, path.read_bytes(), 'image/png')
            else:
                self._send(HTTPStatus.NOT_FOUND, {'status': 'unavailable'})
            return
        summary = REAL_BRIDGE.workflow_status()
        summary.update(mode='real', initialization=initialization_status())
        if REAL_JOB:
            summary['job'] = dict(REAL_JOB)
            root = Path(REAL_JOB['directory']) / 'result'
            if (root / 'progress.json').is_file():
                summary['job'].update(progress=json.loads((root / 'progress.json').read_text()),
                                      runtime=runtime_snapshot(root, REAL_JOB['status'], time.time()))
            if REAL_JOB.get('execution_started_unix_s'):
                elapsed = max(0, REAL_JOB.get('execution_finished_unix_s', time.time()) - REAL_JOB['execution_started_unix_s'])
                status = 'running' if REAL_JOB.get('phase') == 'physical-execution' and REAL_JOB['status'] == 'running' else 'completed'
                if REAL_JOB['status'] in ('failed', 'needs-review'):
                    status = 'failed'
                summary['job']['runtime'] = {'status': REAL_JOB['status'], 'elapsed_s': elapsed,
                    'pipeline_finished': REAL_JOB['status'] != 'running', 'stages': [
                        {'key': 'physical-execution', 'label': '真机执行与原生录制', 'status': status,
                         'elapsed_s': elapsed, 'estimate_s': REAL_JOB['execution_estimate_s'], 'basis': '冻结控制时间线'},
                        {'key': 'export-playback', 'label': '封存校验与生成回放',
                         'status': 'running' if REAL_JOB.get('phase') == 'export-playback' else
                                   'completed' if REAL_JOB['status'] == 'completed' else 'pending',
                         'elapsed_s': None, 'estimate_s': 8, 'basis': '粗估'}]}
        preparation_ready = bool(REAL_BRIDGE.config.get('preparation')) and not summary['trajectory']
        summary['pipeline_configured'] |= preparation_ready
        summary['workflow']['preparation_ready'] = preparation_ready
        values = {'/api/state': summary, '/api/pipeline/status': summary['pipeline'],
                  '/api/pipeline/telemetry': summary['telemetry'],
                  '/api/calibration/profile': summary['calibration_profile'],
                  '/api/scene': summary['scene'], '/api/trajectory/receipt': summary['trajectory']}
        self._send(HTTPStatus.OK if route in values else HTTPStatus.NOT_FOUND,
                   values.get(route) or {'status': 'unavailable'})

    def do_GET(self) -> None:  # noqa: N802
        if not self.local_browser():
            self._send(HTTPStatus.FORBIDDEN, {'error': '工作台 Host 或 Origin 未获允许'})
            return
        route = self.path.split('?', 1)[0]
        if route.startswith('/api/playground/camera/'):
            try:
                from playground_display import rectified_frame
                name, resource = route.removeprefix('/api/playground/camera/').split('/')
                if resource != 'frame.jpg' or not REAL_BRIDGE:
                    raise ValueError('Playground camera frame unavailable')
                self._send(HTTPStatus.OK, rectified_frame(REAL_BRIDGE, name), 'image/jpeg')
            except (OSError, ValueError, KeyError, ImportError) as error:
                self._send(HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(error)})
            return
        if route == '/api/playground' or route.startswith('/api/playground/media/'):
            self.serve_playground(route)
            return
        try:
            mode = self.request_mode()
            self.request_camera()
        except ValueError as error:
            self._send(HTTPStatus.BAD_REQUEST, {'error': str(error)})
            return
        if route == '/api/mode' or route.startswith('/api/real/'):
            if not self.local_browser():
                self._send(HTTPStatus.FORBIDDEN, {'error': 'local workbench only'})
                return
            try:
                if route == '/api/mode':
                    busy = bool(STATE.job and STATE.job.get('status') in ('running', 'needs-review')) or bool(
                        REAL_JOB and REAL_JOB.get('status') in ('running', 'needs-review'))
                    busy = busy or bool(PLAYGROUND and PLAYGROUND.busy)
                    if WORKBENCH_MODE == 'real' and REAL_BRIDGE:
                        owner = REAL_BRIDGE.owner()
                        busy = busy or bool(owner.get('automatic_enabled') or owner.get('acquisition_pending'))
                    self._send(HTTPStatus.OK, {'mode': WORKBENCH_MODE, 'csrf_token': CSRF_TOKEN,
                        'busy': busy})
                elif route == '/api/real/status':
                    self._send(HTTPStatus.OK, REAL_BRIDGE.status() if REAL_BRIDGE else {
                        'configured': False, 'connected': False, 'owner': {}, 'cameras': {},
                        'collection_ready': False, 'blockers': ['真机独立服务尚未配置']})
                elif route.startswith('/api/real/camera/') and REAL_BRIDGE:
                    name, resource = route.removeprefix('/api/real/camera/').split('/')
                    if resource == 'stream.mjpg':
                        with REAL_BRIDGE.camera_stream(name) as stream:
                            self.connection.settimeout(5)
                            self.send_response(HTTPStatus.OK)
                            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                            self.send_header('Cache-Control', 'no-store')
                            self.end_headers()
                            try:
                                while block := stream.read1(65536):
                                    self.wfile.write(block)
                                    self.wfile.flush()
                            except (OSError, TimeoutError):
                                pass
                        return
                    body, mime = REAL_BRIDGE.camera(name, resource)
                    self._send(HTTPStatus.OK, body, mime)
                else:
                    self._send(HTTPStatus.NOT_FOUND, {'error': 'not found'})
            except (OSError, ValueError) as error:
                self._send(HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(error)})
            return
        if self.path.split("?", 1)[0].startswith("/api/playback/"):
            self.serve_playback()
            return
        if mode == 'real' and route.startswith('/api/') and route != '/api/initialization':
            if not self.local_browser():
                self._send(HTTPStatus.FORBIDDEN, {'error': 'local workbench only'})
                return
            try:
                self.real_get(route)
            except (OSError, ValueError, KeyError) as error:
                self._send(HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(error)})
            return
        with LOCK:
            self.get_locked()

    def do_HEAD(self) -> None:  # noqa: N802
        if not self.local_browser():
            self.send_response(HTTPStatus.FORBIDDEN)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        if self.path.split('?', 1)[0].startswith('/api/playground/media/'):
            self.serve_playground(self.path.split('?', 1)[0], head=True)
        elif self.path.split("?", 1)[0].startswith("/api/playback/"):
            self.serve_playback(head=True)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def serve_playback(self, head=False):
        name = self.path.split("?", 1)[0].removeprefix("/api/playback/")
        with LOCK:
            try:
                mode = self.request_mode()
            except ValueError as error:
                self._send(HTTPStatus.BAD_REQUEST, {'error': str(error)})
                return
            path = (REAL_BRIDGE.playback_path(name) if REAL_BRIDGE else None) if mode == 'real' else STATE.playback_path(name)
        if path is None:
            self._send(HTTPStatus.NOT_FOUND, {"error": "playback unavailable"})
            return
        self.serve_video_file(path, head=head)

    def serve_playground(self, route, head=False):
        try:
            service = playground_service()
            if route == '/api/playground':
                self._send(HTTPStatus.OK, service.snapshot() if service else dict(
                    status='idle', busy=False, environment={'ready': False, 'checks': []},
                    plan={'validated': False}, can_execute=False, cameras={}, videos=[],
                    message='真机服务与 Playground 持久化目录尚未配置。'))
                return
            name = route.removeprefix('/api/playground/media/')
            path = service.media(name, parse_qs(urlsplit(self.path).query).get('v', [None])[0]) if service else None
            if path is None:
                self._send(HTTPStatus.NOT_FOUND, {'error': '本轮媒体尚未就绪'})
            elif name.startswith('video-'):
                self.serve_video_file(path, head=head)
            else:
                body = path.read_bytes()
                mime = 'image/png' if body.startswith(b'\x89PNG\r\n\x1a\n') else 'image/jpeg' if body.startswith(b'\xff\xd8') else None
                if mime is None:
                    raise ValueError('首帧产物不是 PNG/JPEG 图像')
                self._send(HTTPStatus.OK, b'' if head else body, mime)
        except (ValueError, OSError, KeyError) as error:
            self._send(HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(error)})

    def serve_video_file(self, path, head=False):
        with path.open("rb") as stream:
            size = path.stat().st_size
            start, end = 0, size - 1
            requested = self.headers.get("Range")
            if requested:
                try:
                    unit, value = requested.split("=", 1)
                    first, last = value.split("-", 1)
                    if unit != "bytes" or "," in value or not (first or last):
                        raise ValueError()
                    if first:
                        start = int(first)
                        end = min(int(last), size - 1) if last else size - 1
                    else:
                        suffix = int(last)
                        if suffix <= 0:
                            raise ValueError()
                        start = max(0, size - suffix)
                    if start < 0 or start >= size or end < start:
                        raise ValueError()
                except ValueError:
                    self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
            self.send_response(HTTPStatus.PARTIAL_CONTENT if requested else HTTPStatus.OK)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(end - start + 1))
            if requested:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if not head:
                stream.seek(start)
                remaining = end - start + 1
                try:
                    while remaining:
                        block = stream.read(min(65536, remaining))
                        if not block:
                            break
                        self.wfile.write(block)
                        remaining -= len(block)
                except (BrokenPipeError, ConnectionResetError):
                    pass

    def get_locked(self):
        route = self.path.split("?", 1)[0]
        if route == "/api/playback":
            self._send(HTTPStatus.OK, STATE.playback_status())
            return
        if route == "/api/initialization":
            self._send(HTTPStatus.OK, initialization_status())
            return
        if route in {"/api/evidence/source.png", "/api/evidence/target.png"}:
            path = STATE.image_evidence(route.rsplit("/", 1)[-1])
            if path:
                self._send(HTTPStatus.OK, path.read_bytes(), "image/png")
            else:
                self._send(HTTPStatus.NOT_FOUND, {"error": "no rendered evidence yet"})
            return
        if route.startswith("/api/live-preview/") and route.endswith(".jpg"):
            side = route.rsplit("/", 1)[-1][:-4]
            live = STATE.live_preview(side, self.request_camera())
            if live:
                self._send(HTTPStatus.OK, live[0].payload, "image/jpeg")
            else:
                self._send(HTTPStatus.NOT_FOUND, {"error": "no live preview yet"})
            return
        if route.startswith("/api/live-preview/") and route.endswith(".json"):
            side = route.rsplit("/", 1)[-1][:-5]
            live = STATE.live_preview(side, self.request_camera())
            self._send(HTTPStatus.OK if live else HTTPStatus.NOT_FOUND, live[1] if live else {"status": "unavailable"})
            return
        if route == "/":
            self._send(HTTPStatus.OK, (ROOT / "index.html").read_text(), "text/html")
            return
        if route == "/api/state":
            self._send(HTTPStatus.OK, STATE.summary())
            return
        if route == "/api/calibration/profile":
            value = ((STATE.current_job_artifact("source.json") or {}).get("calibration_profile") if STATE.unpromoted_job() else STATE.profile)
            self._send(HTTPStatus.OK, value or {"status": "unavailable"})
            return
        if route == "/api/scene":
            value = STATE.summary()["scene"] if STATE.unpromoted_job() else STATE.scene
            self._send(HTTPStatus.OK, value or {"status": "unavailable"})
            return
        if route == "/api/trajectory/receipt":
            self._send(HTTPStatus.OK, (None if STATE.unpromoted_job() else STATE.receipt) or {"status": "unavailable"})
            return
        if route == "/api/pipeline/status":
            self._send(HTTPStatus.OK, STATE.summary()["pipeline"])
            return
        if route == "/api/pipeline/telemetry":
            self._send(HTTPStatus.OK, STATE.telemetry_summary())
            return
        if route == "/api/pipeline/plan":
            if "view=latest" in self.path or STATE.unpromoted_job():
                plan = STATE.latest_visual_plan()
                self._send(HTTPStatus.OK, plan or {"status": "unavailable", "validated": False})
            else:
                self._send(HTTPStatus.OK, STATE.pipeline.plan or {"status": "unavailable"})
            return
        self._send(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        with LOCK:
            self.post_locked()

    def local_browser(self):
        # Explicit authorities prevent DNS rebinding; never trust forwarded headers.
        allowed = {f'{host}:{self.server.server_port}' for host in
                   ('127.0.0.1', 'localhost', *getattr(self.server, 'allowed_hosts', ()))}
        hosts = self.headers.get_all('Host', [])
        origins = self.headers.get_all('Origin', [])
        return (len(hosts) == 1 and hosts[0] in allowed and len(origins) <= 1
                and (not origins or origins[0] == 'http://' + hosts[0]))

    def post_locked(self):
        global WORKBENCH_MODE
        try:
            if not self.local_browser() or self.headers.get_content_type() != 'application/json':
                self._send(HTTPStatus.FORBIDDEN, {'error': '工作台 Host、Origin 或 JSON 类型未获允许'})
                return
            loopback = self.headers.get('Host') in {
                f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
            if (not loopback or getattr(self.server, 'allowed_hosts', ())) and not secrets.compare_digest(self.headers.get('X-Workbench-CSRF', ''), CSRF_TOKEN):
                self._send(HTTPStatus.FORBIDDEN, {'error': '需要当前工作台的 CSRF 凭据'})
                return
            payload = self._json()
            if not isinstance(payload, dict):
                raise ValueError("JSON object required")
            if self.path == '/api/playground':
                if not secrets.compare_digest(self.headers.get('X-Workbench-CSRF', ''), CSRF_TOKEN):
                    self._send(HTTPStatus.FORBIDDEN, {'error': '需要当前工作台的 CSRF 凭据'})
                    return
                service = playground_service()
                if service is None:
                    raise ValueError('真机服务与 Playground 持久化目录尚未配置')
                self._send(HTTPStatus.ACCEPTED, service.start(payload))
                return
            if PLAYGROUND and PLAYGROUND.busy and self.path in (
                    '/api/mode', '/api/real/control', '/api/pipeline/run', '/api/pipeline/diagnostic'):
                if self.path != '/api/real/control' or payload != {'action': 'stop'}:
                    raise ValueError('Playground 正在运行；请等待结束，紧急停止仍然可用')
            if self.path in ('/api/mode', '/api/real/control'):
                if (not self.local_browser() or self.headers.get_content_type() != 'application/json'
                        or not secrets.compare_digest(self.headers.get('X-Workbench-CSRF', ''), CSRF_TOKEN)):
                    self._send(HTTPStatus.FORBIDDEN, {'error': '需要当前本地工作台的 CSRF 凭据'})
                    return
                if self.path == '/api/mode':
                    if set(payload) != {'mode'} or payload['mode'] not in ('dual_dsw', 'real'):
                        raise ValueError('unknown workbench mode')
                    if STATE.job and STATE.job.get('status') in ('running', 'needs-review'):
                        raise ValueError('采集运行或终态待核验期间不能切换模式')
                    if REAL_JOB and REAL_JOB.get('status') in ('running', 'needs-review'):
                        raise ValueError('真实场景规划运行或终态待核验期间不能切换模式')
                    if REAL_BRIDGE and WORKBENCH_MODE == 'real':
                        owner = REAL_BRIDGE.owner()
                        if owner.get('automatic_enabled') or owner.get('acquisition_pending'):
                            raise ValueError('真机执行/连接期间不能切换模式')
                    WORKBENCH_MODE = payload['mode']
                    result = {'mode': WORKBENCH_MODE, 'csrf_token': CSRF_TOKEN, 'busy': False}
                else:
                    if WORKBENCH_MODE != 'real' or REAL_BRIDGE is None:
                        raise ValueError('请先配置并切换至真机接入模式')
                    if payload.get('action') == 'confirm':
                        if REAL_STATE_DIR is None or REAL_JOB and REAL_JOB.get('status') in ('running', 'needs-review'):
                            raise ValueError('真机执行需要持久化目录，且不能有未核验运行')
                        _, confirmed_plan = REAL_BRIDGE.inputs()
                        previous_recording = (REAL_BRIDGE.owner().get('recording') or {}).get('directory')
                    result = REAL_BRIDGE.control(payload)
                    if payload.get('action') == 'confirm':
                        watch_confirmed_execution(confirmed_plan, previous_recording)
                self._send(HTTPStatus.OK, result)
                return
            if self.path in {"/api/pipeline/run", "/api/pipeline/diagnostic"}:
                requested_mode = payload.pop('mode', WORKBENCH_MODE)
                if requested_mode != WORKBENCH_MODE:
                    raise ValueError('模式已切换，请刷新后重试')
                if WORKBENCH_MODE == 'real':
                    if not self.local_browser() or not secrets.compare_digest(self.headers.get('X-Workbench-CSRF', ''), CSRF_TOKEN):
                        self._send(HTTPStatus.FORBIDDEN, {'error': '需要当前本地工作台的 CSRF 凭据'})
                        return
                    if payload or REAL_BRIDGE is None:
                        raise ValueError('真机独立服务尚未配置，或请求包含未知字段')
                    if self.path.endswith('/diagnostic'):
                        raise ValueError('真机强制采集尚未完成独立验收；不能复用双 DSW 碰撞旁路')
                    if REAL_BRIDGE.config.get('preparation'):
                        if 'acquisition' in REAL_BRIDGE.config['preparation']:
                            self._send(HTTPStatus.ACCEPTED, start_real_preparation())
                            return
                        try:
                            REAL_BRIDGE.inputs()
                        except (OSError, ValueError, KeyError):
                            self._send(HTTPStatus.ACCEPTED, start_real_preparation())
                            return
                    self._send(HTTPStatus.ACCEPTED, REAL_BRIDGE.prepare_collection())
                    return
                if payload or not PIPELINE_CONFIG or not STATE.directory:
                    raise ValueError("启动需要预先冻结的 --pipeline-config 和 --state-dir；请求体必须为空对象")
                if STATE.job and STATE.job.get("status") in {"running", "needs-review"}:
                    self._send(HTTPStatus.CONFLICT, {"error": "已有采集运行或待核验的中断运行"})
                    return
                initialization = initialization_status()
                if initialization.get("status") != "ready":
                    self._send(HTTPStatus.SERVICE_UNAVAILABLE,
                               {"error": "检测器尚未就绪：" + str(initialization.get("error") or initialization.get("message") or initialization.get("stage")),
                                "initialization": initialization})
                    return
                diagnostic = self.path.endswith("/diagnostic")
                if diagnostic:
                    if not STATE.job or not STATE.job.get("directory") or not STATE.latest_visual_plan():
                        raise ValueError("当前没有可用于强制诊断的失败轨迹")
                output = (STATE.directory / "jobs" / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid4().hex[:8])).resolve()
                output.mkdir(parents=True, exist_ok=False)
                config = output / "config.json"
                # Use the running workbench's pinned code/service configuration.
                # A historical job contributes only the frozen diagnostic input.
                base = json.loads(PIPELINE_CONFIG.read_text())
                template_root = base["remote_run_root"]
                old_root = template_root
                if diagnostic:
                    previous_config = Path(STATE.job["directory"]) / "config.json"
                    if not previous_config.is_file():
                        raise ValueError("强制诊断需要上一轮的冻结配置来定位输入")
                    old_root = json.loads(previous_config.read_text())["remote_run_root"]
                planner=base.get('planner_service')
                remote_root = (planner['allowed_output_root']+'/'+('diagnostic-' if diagnostic else 'run-')+uuid4().hex
                    if planner else template_root.rstrip("/") + ("-forced-diagnostic-" if diagnostic else "-run-") + uuid4().hex)
                base["remote_run_root"] = remote_root
                contract = json.loads(Path(base["contract_path"]).read_text())
                if diagnostic:
                    base.update(force_diagnostic=True, record_raw=False, diagnostic_input_root=old_root,
                                remote_run_root=remote_root)
                    readonly = contract.setdefault("filesystem", {}).setdefault("readonly_roots", [])
                    if old_root not in readonly:
                        readonly.append(old_root)
                # Move only the per-run output grant; preserve the frozen
                # detector service root and its session/identity constraints.
                contract["filesystem"]["writable_roots"] = [
                    remote_root if root == template_root else root
                    for root in contract["filesystem"]["writable_roots"]]
                contract.setdefault("identity", {})["experiment_id"] = output.name
                contract_path = output / "contract.json"
                base["contract_path"] = str(contract_path)
                config.write_text(json.dumps(base, ensure_ascii=False, indent=2))
                if "config_sha256" in contract.get("provenance", {}):
                    contract["provenance"]["config_sha256"] = {"config.json": hashlib.sha256(config.read_bytes()).hexdigest()}
                contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2))
                STATE.job = {"status": "running", "directory": str(output), "mode": "diagnostic" if diagnostic else "normal"}
                STATE.persist()
                Thread(target=run_pipeline_job, args=(config, output), daemon=True).start()
                self._send(HTTPStatus.ACCEPTED, STATE.job)
                return
            if WORKBENCH_MODE == 'real':
                raise ValueError('真机观测、标定与回执须来自绑定的采集服务，不能使用双 DSW 注入接口')
            if STATE.job and STATE.job.get("status") == "running":
                self._send(HTTPStatus.CONFLICT, {"error": "采集运行中，暂不能替换 profile、场景或执行回执"})
                return
            if self.path == "/api/validate/physical14":
                validate_physical14(payload)
                result = {"valid": True, "schema_version": "physical14.v1"}
            elif self.path == "/api/command":
                maximum = STATE.profile["gripper"]["max_opening_mm"] if STATE.profile else None
                validate_command(payload, gripper_max_mm=maximum)
                if not payload["dry_run"]:
                    self._send(HTTPStatus.CONFLICT, {"error": "hardware_required", "detail": "real adapter is intentionally not connected"})
                    return
                STATE.last_action = payload["action"]
                result = {"accepted": True, "dry_run": True, "applied": True, "adapter": "workbench-dry-run"}
            elif self.path == "/api/calibration":
                if "profile" not in payload:
                    validate_boundary("calibration", payload)
                    raise ValueError("ready calibration requires a versioned profile payload")
                profile = validate_profile(payload["profile"])
                STATE.profile = profile
                thresholds = profile["divergence_thresholds"]
                STATE.window = ObservationWindow(
                    profile, [],
                    cup_motion_threshold_m=thresholds.get("cup_motion_threshold_m"),
                    cup_position_jitter_m=thresholds.get("cup_position_jitter_m"),
                    cup_motion_threshold_source="calibration_profile" if thresholds.get("cup_motion_threshold_m") is not None
                    else "unresolved")
                STATE.scene = STATE.receipt = None
                STATE.pipeline = DualGpuPipeline()
                result = {"accepted": True, "profile_id": profile["profile_id"], "schema_version": profile["schema_version"],
                          "profile_digest": _digest(profile)}
            elif self.path == "/api/online-detection":
                if STATE.window is None:
                    raise ValueError("calibration profile must be ready before detection collection")
                snapshot = payload.get("snapshot", payload)
                result = STATE.window.add(snapshot)
                if result.get("reobserve_required"):
                    # A new cup pose invalidates both the detected scene and every
                    # trajectory planned from it. The next accepted scene must
                    # rebuild the target lane from a fresh five-second window.
                    STATE.scene = STATE.receipt = None
                    STATE.pipeline = DualGpuPipeline()
                    result = {**result, "scene": None,
                              "scene_digest": None,
                              "message": "纸杯发生超阈值位移，旧场景和轨迹已废弃；请重新完成 5 秒观测"}
                elif result["ready"]:
                    scene = STATE.window.finalize()
                    if STATE.scene != scene:
                        STATE.receipt = None
                    STATE.scene = scene
                    result = {**result, "scene": STATE.scene, "scene_digest": STATE.scene["scene_digest"]}
            elif self.path == "/api/trajectory/freeze":
                if not STATE.profile or not STATE.scene:
                    raise ValueError("calibration profile and 5 second scene are required")
                STATE.receipt = freeze_receipt(payload, STATE.profile, STATE.scene)
                result = STATE.receipt
            elif self.path == "/api/pipeline/source":
                result = STATE.pipeline.ingest_source(payload)
                STATE.profile = STATE.pipeline.source["calibration_profile"]
                STATE.scene = STATE.pipeline.source["scene"]
                STATE.window = None
                STATE.receipt = None
            elif self.path == "/api/pipeline/plan":
                result = STATE.pipeline.accept_plan(payload)
            elif self.path == "/api/pipeline/execution":
                result = STATE.pipeline.record_execution(payload)
                STATE.last_action = payload["measured_states"][-1]
            else:
                self._send(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return
            STATE.persist()
            self._send(HTTPStatus.OK, result)
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self._send(HTTPStatus.BAD_REQUEST, {"error": str(exc)})

    def log_message(self, fmt: str, *args: object) -> None:
        print(fmt % args, file=sys.stderr)


def main() -> None:
    import argparse
    import signal

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8119)
    parser.add_argument('--allow-host', action='append', default=[],
                        help='explicit browser hostname or IPv4 address (no port); repeatable')
    parser.add_argument("--state-dir", type=Path, help="task-owned durable state and rendered evidence")
    parser.add_argument("--pipeline-config", type=Path, help="operator-frozen dual-DSW configuration; enables run action")
    parser.add_argument('--real-config', type=Path, help='independent real services over loopback SSH forwards')
    args = parser.parse_args()
    import re
    for host in args.allow_host:
        if not re.fullmatch(r'[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?', host) or host == '0.0.0.0':
            parser.error('--allow-host requires an explicit hostname or IPv4 address')
    if args.host not in ('127.0.0.1', 'localhost') and not args.allow_host:
        parser.error('non-loopback binding requires explicit --allow-host')
    global PIPELINE_CONFIG, DETECTOR_MANAGER, PLANNER_MANAGER, INITIALIZATION_ERROR, REAL_BRIDGE, REAL_JOB, REAL_STATE_DIR, PLAYGROUND
    PLAYGROUND = None
    if args.real_config:
        from real_bridge import RealBridge
        REAL_BRIDGE = RealBridge(json.loads(args.real_config.read_text()))
    real_template = ((REAL_BRIDGE.config.get('preparation') or {}).get('pipeline_config') if REAL_BRIDGE else None)
    PIPELINE_CONFIG = args.pipeline_config.resolve() if args.pipeline_config else Path(real_template).resolve() if real_template else None
    if PIPELINE_CONFIG:
        if not args.state_dir:
            parser.error("--pipeline-config requires --state-dir")
        json.loads(PIPELINE_CONFIG.read_text())
    if args.state_dir:
        STATE.restore(args.state_dir)
        if REAL_BRIDGE:
            REAL_STATE_DIR = args.state_dir.resolve() / 'real-mode'
            saved = REAL_STATE_DIR / 'state.json'
            if saved.is_file():
                value = json.loads(saved.read_text())
                REAL_JOB = value.get('job')
                if REAL_JOB and REAL_JOB.get('status') == 'running':
                    REAL_JOB.update(status='needs-review', error='服务重启；先核验真实规划远端终态，不自动重启')
                for key, path in value.get('artifacts', {}).items():
                    if key in ('source_path', 'plan_path', 'hardware_binding', 'artifact_dir'):
                        if path:
                            REAL_BRIDGE.config[key] = path
                        else:
                            REAL_BRIDGE.config.pop(key, None)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.allowed_hosts = tuple(args.allow_host)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f"Rule Based workbench: http://{args.host}:{args.port}/")
    try:
        if PIPELINE_CONFIG:
            try:
                from detector_service import DetectorService
                PIPELINE_CONFIG = shared_startup_config(PIPELINE_CONFIG, STATE.directory, REAL_BRIDGE.config if REAL_BRIDGE else None)
                DETECTOR_MANAGER = DetectorService()
                DETECTOR_MANAGER.start(PIPELINE_CONFIG, STATE.directory)
                if json.loads(PIPELINE_CONFIG.read_text()).get('planner_service'):
                    from planner_service import PlannerService
                    PLANNER_MANAGER = PlannerService()
                    PLANNER_MANAGER.start(PIPELINE_CONFIG,STATE.directory)
            except Exception as error:
                INITIALIZATION_ERROR = str(error)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if DETECTOR_MANAGER is not None:
            DETECTOR_MANAGER.close()
        if PLANNER_MANAGER is not None:
            PLANNER_MANAGER.close()


if __name__ == "__main__":
    main()
