"""Derived browser videos; sealed raw payloads are always read-only."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

DEVICES = ("base_0", "left_wrist_0", "right_wrist_0")
FILES = tuple(f"{side}-{device}.mp4" for side in ("source", "target") for device in DEVICES)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate(root, timeline_digest, *, run_id=None):
    root = Path(root)
    manifest = json.loads((root / "manifest.json").read_text())
    if (manifest.get("schema_version") != "workbench_playback.v1" or manifest.get("clock") != "simulation"
            or manifest.get("timeline_digest") != timeline_digest
            or (run_id is not None and manifest.get("run_id") != run_id)
            or set(manifest.get("files", {})) != set(FILES)):
        raise ValueError("playback identity or six-camera file set mismatch")
    validate_videos(root, manifest)
    return manifest


def validate_videos(root, manifest):
    for name, record in manifest["files"].items():
        path = root / name
        if path.is_symlink() or path.stat().st_size != record["size_bytes"] or sha256(path) != record["sha256"]:
            raise ValueError("playback file integrity mismatch: " + name)
        probe = record["probe"]
        if (probe["codec_name"], probe["pix_fmt"], probe["width"], probe["height"], probe["r_frame_rate"]) != (
                "h264", "yuv420p", 640, 480, "30/1") or int(probe["nb_read_frames"]) != manifest["camera_frames"]:
            raise ValueError("playback video format/count mismatch: " + name)


def validate_target(root, source, plan):
    root = Path(root)
    manifest = json.loads((root / 'manifest.json').read_text())
    if (manifest.get('schema_version') != 'physical_target_playback.v1' or manifest.get('evidence_source') != 'simulation'
            or manifest.get('source_run_id') != source['run_id'] or manifest.get('plan_run_id') != plan['run_id']
            or manifest.get('timeline_digest') != plan['timeline_digest']
            or set(manifest.get('files', {})) != {f'target-{device}.mp4' for device in DEVICES}):
        raise ValueError('physical target playback identity or three-camera file set mismatch')
    validate_videos(root, manifest)
    return manifest


def export(run, *, target_only=False):
    run = Path(run).resolve(strict=True)
    plan = json.loads((run / "plan.json").read_text())
    physical_source = None
    if target_only:
        from real_contract import validate_real_plan
        physical_source = json.loads((run / 'detection-bridge/source.json').read_text())
        binding = json.loads((run / 'detection-bridge/hardware-binding.json').read_text())
        validate_real_plan(plan, physical_source, binding)
    timeline = plan["timeline_digest"]
    k = sum(plan["native_substeps"])
    frame_count = k // 8 + 1
    inputs, provenance = [], {}
    sides = (("target", "dsw-scene"),) if target_only else (("source", "simulated-real"), ("target", "dsw-scene"))
    for side, lane in sides:
        receipt = json.loads((run / f"replay-{side}/execution.json").read_text())
        if target_only and (receipt.get('source_run_id') != physical_source['run_id']
                            or receipt.get('plan_run_id') != plan['run_id']):
            raise ValueError('target execution belongs to another physical source or plan')
        recording = receipt["recording"]
        raw = Path(recording["directory"]).resolve(strict=True)
        manifest_path = raw / "manifests/episode.json"
        manifest = json.loads(manifest_path.read_text())
        if (not raw.is_relative_to(run / f"replay-{side}") or receipt.get("side") != lane
                or receipt.get("status") != "completed" or receipt.get("timeline_digest") != timeline
                or recording.get("timeline_digest") != timeline
                or recording.get("capture_status") != "complete" or recording.get("validation") != "passed"
                or recording.get("manifest_sha256") != sha256(manifest_path)
                or manifest.get("schema_version") != "sim_raw_dataset/v1"
                or manifest.get("batch_id") != receipt.get("run_id")
                or manifest.get("capture_status") != "complete" or manifest.get("outcome") != "success"
                or manifest.get("validation", {}).get("status") != "passed"
                or manifest.get("final_physics_step") != k):
            raise ValueError("raw receipt/manifest identity or sealed-success mismatch: " + side)
        metadata = json.loads((raw / "camera/metadata.json").read_text())
        if (set(metadata) != {d + "_rgb" for d in DEVICES}
                or any(metadata[d + "_rgb"]["frames"] != frame_count for d in DEVICES)
                or recording.get("camera_frames") != dict.fromkeys(DEVICES, frame_count)):
            raise ValueError("raw camera count mismatch: " + side)
        sealed_files = {item["path"]: item for item in manifest["files"]}
        for device in DEVICES:
            relative = f"camera/{device}_rgb.mp4"
            source = raw / relative
            pin = sealed_files[relative]
            if source.is_symlink() or source.stat().st_size != pin["size_bytes"] or sha256(source) != pin["sha256"]:
                raise ValueError("sealed raw video changed: " + relative)
            inputs.append((source, f"{side}-{device}.mp4"))
        provenance[side] = {"directory": str(raw), "manifest_sha256": sha256(manifest_path),
                            "execution_sha256": sha256(run / f"replay-{side}/execution.json")}
    output = run / ("target-playback" if target_only else "playback")
    output.mkdir(exist_ok=False)
    files = {}
    for source, name in inputs:
        target = output / name
        subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-n", "-threads", "2", "-i", str(source),
                        "-an", "-c:v", "libx264", "-threads", "2", "-preset", "fast", "-crf", "24",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(target)], check=True, timeout=300)
        probe = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0", "-show_streams", "-of", "json",
            str(target)], timeout=120))["streams"]
        if len(probe) != 1:
            raise ValueError("one playback video stream required")
        files[name] = {"sha256": sha256(target), "size_bytes": target.stat().st_size, "probe": probe[0]}
    manifest = {"schema_version": "workbench_playback.v1", "run_id": run.name, "clock": "simulation",
                "timeline_digest": timeline, "final_physics_step": k, "camera_frames": frame_count,
                "raw": provenance, "files": files}
    if target_only:
        manifest.update(schema_version='physical_target_playback.v1',evidence_source='simulation',
                        source_run_id=physical_source['run_id'],plan_run_id=plan['run_id'])
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if target_only:
        return validate_target(output, physical_source, plan)
    return validate(output, timeline, run_id=run.name)


def export_target(run):
    """Only the validated simulation target; physical videos are recorded later."""
    return export(run, target_only=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument('--target-only', action='store_true')
    args = parser.parse_args()
    print(json.dumps(export(args.run_root, target_only=args.target_only)))
