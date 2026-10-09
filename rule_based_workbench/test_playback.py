"""CPU exporter boundaries using fake encoders; no raw or GPU mutation."""
import json
import shutil
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from playback import DEVICES, FILES, export, export_target, sha256, validate, validate_target
from integration import _digest
from test_frozen_executor import fixture as physical_fixture


def fixture(root):
    (root / "plan.json").write_text(json.dumps({"timeline_digest": "timeline", "native_substeps": [8, 5]}))
    for side, lane in (("source", "simulated-real"), ("target", "dsw-scene")):
        raw = root / f"replay-{side}/raw/episode"
        (raw / "camera").mkdir(parents=True)
        (raw / "manifests").mkdir()
        pins = []
        for device in DEVICES:
            path = raw / f"camera/{device}_rgb.mp4"
            path.write_bytes(b"sealed-hevc")
            pins.append({"path": str(path.relative_to(raw)), "sha256": sha256(path), "size_bytes": path.stat().st_size})
        (raw / "camera/metadata.json").write_text(json.dumps({d + "_rgb": {"frames": 2} for d in DEVICES}))
        manifest = {"schema_version": "sim_raw_dataset/v1", "batch_id": "receipt-" + side,
                    "capture_status": "complete", "outcome": "success", "validation": {"status": "passed"},
                    "final_physics_step": 13, "files": pins}
        manifest_path = raw / "manifests/episode.json"
        manifest_path.write_text(json.dumps(manifest))
        receipt = {"side": lane, "run_id": "receipt-" + side, "status": "completed", "timeline_digest": "timeline",
                   "recording": {"directory": str(raw), "timeline_digest": "timeline", "capture_status": "complete",
                                 "validation": "passed", "manifest_sha256": sha256(manifest_path),
                                 "camera_frames": dict.fromkeys(DEVICES, 2)}}
        (root / f"replay-{side}/execution.json").write_text(json.dumps(receipt))


def main():
    with TemporaryDirectory() as temp:
        root = Path(temp)
        fixture(root)
        before = {str(p): p.read_bytes() for p in root.rglob("*") if p.is_file()}
        def encode(argv, **kwargs):
            assert "-n" in argv and "libx264" in argv and "yuv420p" in argv and "+faststart" in argv
            assert argv[argv.index("-threads") + 1] == "2"
            Path(argv[-1]).write_bytes(b"browser-h264")
        probe = {"codec_name": "h264", "pix_fmt": "yuv420p", "width": 640, "height": 480,
                 "r_frame_rate": "30/1", "nb_read_frames": "2"}
        with patch("playback.subprocess.run", side_effect=encode), patch(
                "playback.subprocess.check_output", return_value=json.dumps({"streams": [probe]}).encode()):
            result = export(root)
            assert set(result["files"]) == set(FILES) and result["camera_frames"] == 2
            assert all(Path(p).read_bytes() == content for p, content in before.items())
            try:
                export(root)
            except FileExistsError:
                pass
            else:
                raise AssertionError("playback output overwritten")
        validate(root / "playback", "timeline", run_id=root.name)
        for timeline in ("other",):
            try:
                validate(root / "playback", timeline)
            except ValueError:
                pass
            else:
                raise AssertionError("wrong timeline accepted")
        (root / "playback" / FILES[0]).write_bytes(b"corrupt")
        try:
            validate(root / "playback", "timeline")
        except ValueError:
            pass
        else:
            raise AssertionError("corrupt video accepted")
    for mutation in ("incomplete", "manifest", "timeline", "camera", "raw_video"):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            fixture(root)
            path = root / "replay-source/execution.json"
            receipt = json.loads(path.read_text())
            if mutation == "incomplete": receipt["recording"]["capture_status"] = "recording"
            if mutation == "manifest": receipt["recording"]["manifest_sha256"] = "wrong"
            if mutation == "timeline": receipt["timeline_digest"] = "other"
            if mutation == "camera": receipt["recording"]["camera_frames"]["base_0"] = 1
            if mutation == "raw_video":
                (root / "replay-source/raw/episode/camera/base_0_rgb.mp4").write_bytes(b"corrupt")
            path.write_text(json.dumps(receipt))
            try:
                export(root)
            except ValueError:
                pass
            else:
                raise AssertionError("accepted " + mutation)
            assert not (root / "playback").exists()
    with TemporaryDirectory() as temp:
        root=Path(temp); fixture(root)
        shutil.rmtree(root/'replay-source')
        source,plan=physical_fixture(.001)
        source={k:v for k,v in source.items() if k not in ('lane','adapter','gpu','instance_id')}
        source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',
                      robot_id='fixture',camera_serials=dict(head='h',left='l',right='r'))
        from test_joint_space import physical_fixture as freeze_fixture
        freeze_fixture(plan,source)
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',camera_serials=source['camera_serials'],
                     source_digest=_digest(source),calibration_profile_digest=_digest(source['calibration_profile']))
        (root/'plan.json').write_text(json.dumps(plan)); (root/'detection-bridge').mkdir()
        for name,value in [('source.json',source),('hardware-binding.json',binding)]:
            (root/'detection-bridge'/name).write_text(json.dumps(value))
        manifest_path=root/'replay-target/raw/episode/manifests/episode.json'
        manifest=json.loads(manifest_path.read_text()); manifest['final_physics_step']=20
        manifest_path.write_text(json.dumps(manifest))
        metadata=root/'replay-target/raw/episode/camera/metadata.json'
        metadata.write_text(json.dumps({d+'_rgb':{'frames':3} for d in DEVICES}))
        receipt_path=root/'replay-target/execution.json'; receipt=json.loads(receipt_path.read_text())
        receipt.update(timeline_digest=plan['timeline_digest'],source_run_id=source['run_id'],plan_run_id=plan['run_id'])
        receipt['recording'].update(timeline_digest=plan['timeline_digest'],manifest_sha256=sha256(manifest_path),camera_frames=dict.fromkeys(DEVICES,3))
        receipt_path.write_text(json.dumps(receipt))
        with patch('playback.subprocess.run',side_effect=encode),patch('playback.subprocess.check_output',return_value=json.dumps({'streams':[{**probe,'nb_read_frames':'3'}]}).encode()):
            target=export_target(root)
        assert target['schema_version']=='physical_target_playback.v1'
        assert set(target['files'])=={f'target-{d}.mp4' for d in DEVICES}
        assert target['source_run_id']==source['run_id'] and target['plan_run_id']==plan['run_id']
        assert not (root/'playback').exists() and not (root/'replay-source').exists()
        validate_target(root/'target-playback',source,plan)
        try: validate_target(root/'target-playback',{**source,'run_id':'other'},plan)
        except ValueError: pass
        else: raise AssertionError('different physical source accepted')
    print("Playback sealed-input, six-camera/physical-target export, no-overwrite, hash and timeline: PASS")


if __name__ == "__main__":
    main()
