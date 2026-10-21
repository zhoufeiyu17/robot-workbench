"""CPU fake-SSH checks; never inference, simulation, remote execution or synchronization evidence."""
from copy import deepcopy
import json
import os
from pathlib import Path
import shlex
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

from integration import PIPELINE_LANES, _digest
from run_pipeline import PATH_KEYS, bounded_command, load_config, run, verify, verify_observation, verify_detection, python_command, digest
from test_app import fixture_messages
from test_harness import rejects
from playback import FILES as PLAYBACK_FILES, sha256
from live_preview import PREVIEW_CAMERAS, PreviewPublisher, camera_frame_path, read_frame


def main():
    import subprocess
    large = json.dumps({f"/long/candidate/path/{i}": str(i)*64 for i in range(1000)})
    tricky = "spaces ' quotes \n and $(do-not-run)"
    command = python_command('import sys,json; print(json.dumps(sys.argv[1:]))', large, tricky)
    assert len(command.encode()) < 32768
    assert json.loads(subprocess.check_output(shlex.split(command), text=True)) == [large, tricky]
    import hashlib
    pins={f'/task/assets/robot/geometry/{i:04d}.json':hashlib.sha256(str(i).encode()).hexdigest() for i in range(1200)}
    repeated=json.dumps(dict(provenance={'input_sha256':pins},planner_inputs=pins))
    command=python_command('import sys,hashlib; print(hashlib.sha256(sys.argv[1].encode()).hexdigest())',repeated)
    assert len(command.encode())<120000  # Linux per-argument limit, including SSH/shell wrappers.
    assert subprocess.check_output(shlex.split(command),text=True).strip()==hashlib.sha256(repeated.encode()).hexdigest()
    with TemporaryDirectory(prefix="dual-dsw-cli-") as temp:
        root = Path(temp)
        remote = "/mnt/data/feiyu.zhou/runs/robot_sim/test-unique"
        contract = {"schema_version": "anyverse.dsw-codex-experiment.v1",
            "filesystem": {"readonly_roots": ["/remote/input"], "writable_roots": [remote],
                           "allow_overwrite": False, "allow_delete": False},
            "resources": {"lanes": {v["instance_id"]: [v["gpu"]] for v in PIPELINE_LANES.values()},
                "max_gpu_count_per_instance": 1, "max_cpu_threads": 4,
                "require_gpu_idle_check": True, "max_wall_time_minutes": 20}}
        contract_path = root / "contract.json"
        contract_path.write_text(json.dumps(contract))
        config = {key: "/remote/input/" + key for key in PATH_KEYS}
        config.update(contract_path=str(contract_path), finger_envelope_sha256="a"*64,
                      opening_mm=83.0, remote_run_root=remote, record_raw=True,
                      prompt_file="/remote/input/prompt.json")
        config_path = root / "config.json"
        config_path.write_text(json.dumps(config))
        bad = deepcopy(contract)
        bad["provenance"] = {"input_sha256": {"/not-authorized/input": "a"*64}}
        contract_path.write_text(json.dumps(bad))
        rejects(load_config, config_path)
        contract_path.write_text(json.dumps(contract))
        dry = run(config_path, root / "dry", dry_run=True)
        assert dry["status"] == "dry-run" and not (root / "dry").exists()
        stages = {s["name"]: s for s in dry["stages"]}
        for stage in stages.values():
            argv = stage["argv"]
            assert "StrictHostKeyChecking=yes" in argv and "BatchMode=yes" in argv
            assert argv[-1].startswith("timeout -k 30 600")
            if stage["output"]:
                assert shlex.split(argv[-1])[4:6] == ["bash", "-c"], "login banners corrupt artifacts"
            if stage["side"] == "simulated-real":
                assert "739" in argv and "HostKeyAlias=[47.116.111.83]:739" in argv
            else:
                assert "741" in argv and "HostKeyAlias=[47.116.111.83]:741" in argv
        for name in ("geometry", "place", "replay-target"):
            assert "CUDA_VISIBLE_DEVICES=0" in stages[name]["argv"][-1]
        for name in ("observe", "detect", "replay-source"):
            assert "CUDA_VISIBLE_DEVICES=0" in stages[name]["argv"][-1]
        assert "owned_process.py --timeout 570" in stages["place"]["argv"][-1]
        candidate = deepcopy(contract)
        candidate["resources"].update(max_wall_time_minutes=30, max_planning_time_seconds=1200)
        contract_path.write_text(json.dumps(candidate))
        extended = run(config_path, root / "extended", dry_run=True)
        place = next(s for s in extended["stages"] if s["name"] == "place")
        assert place["timeout_s"] == 1230 and "owned_process.py --timeout 1200" in place["argv"][-1]
        shortened, seconds = bounded_command(place, 200)
        assert seconds == 155 and shlex.split(shortened[-1])[3] == "155"
        for invalid in (0, -1, True, 1800):
            candidate["resources"]["max_planning_time_seconds"] = invalid
            contract_path.write_text(json.dumps(candidate))
            rejects(load_config, config_path)
        contract_path.write_text(json.dumps(contract))
        assert "--render-gpu 0" in stages["place"]["argv"][-1]
        assert "os.sched_setaffinity(0,cores)" in stages["place"]["argv"][-1]
        assert "--observation-seconds 8" in stages["observe"]["argv"][-1]
        motiongen_config=dict(config,planning_backend='motiongen',curobo_root='/remote/input/curobo')
        resumed=dict(motiongen_config,observation_input_root='/remote/input/completed/capture')
        config_path.write_text(json.dumps(resumed))
        resumed_stages=run(config_path,root/'resume-dry',dry_run=True)['stages']
        assert not any(p['name']=='observe' for p in resumed_stages)
        assert any(p['name']=='observe-verify' for p in resumed_stages)
        assert '/remote/input/completed/capture/rgbd-manifest.json' in next(p['argv'][-1] for p in resumed_stages if p['name']=='detect')
        config_path.write_text(json.dumps(dict(resumed,detection_input_root='/remote/input/detected')))
        detected_stages=run(config_path,root/'detect-resume-dry',dry_run=True)['stages']
        assert not any(p['name'] in ('observe','detect') for p in detected_stages)
        assert any(p['name']=='detect-verify' for p in detected_stages)
        assert '--detection-dir /remote/input/detected' in next(p['argv'][-1] for p in detected_stages if p['name']=='bridge')
        config_path.write_text(json.dumps(dict(resumed,planned_input_root='/remote/input/accepted',replay_profile_stop_after_ticks=16)))
        plan_stages=run(config_path,root/'plan-resume-dry',dry_run=True)['stages']
        assert any(p['name']=='resume-plan' for p in plan_stages)
        assert not any(p['name'] in ('observe','detect','bridge','motiongen-scene','motiongen-plan','motiongen-publish') for p in plan_stages)
        assert all('--profile-stop-after-ticks 16' in p['argv'][-1] for p in plan_stages if p['group']=='replay')
        config_path.write_text(json.dumps(dict(resumed,replay_profile_stop_after_ticks=True)))
        rejects(load_config,config_path)
        config_path.write_text(json.dumps(dict(motiongen_config,detection_input_root='/remote/input/detected')))
        rejects(load_config,config_path)
        config_path.write_text(json.dumps(dict(resumed,observation_input_root=remote+'/capture')))
        rejects(load_config,config_path)
        config_path.write_text(json.dumps(motiongen_config))
        motiongen_stages=run(config_path,root/'motiongen-dry',dry_run=True)['stages']
        for phase in motiongen_stages:
            if phase['group']=='replay':
                assert '--independent-replay' in phase['argv'][-1] and '--barrier' not in phase['argv'][-1]
        limited=deepcopy(contract)
        limited['resources']['stage_wall_seconds']={'observe':600,'detect':300,'motiongen-scene':300,'motiongen-plan':180,'export-playback':120}
        contract_path.write_text(json.dumps(limited))
        bounded=run(config_path,root/'stage-budgets-dry',dry_run=True)['stages']
        for phase in bounded:
            if phase['name'] in limited['resources']['stage_wall_seconds']:
                assert phase['timeout_s']+30<=limited['resources']['stage_wall_seconds'][phase['name']]
        assert 'owned_process.py --timeout 130' in next(p['argv'][-1] for p in bounded if p['name']=='motiongen-plan')
        contract_path.write_text(json.dumps(contract))
        config_path.write_text(json.dumps(config))

        # A failed detector may resume its completed observation, never a changed or unpinned input.
        completed=root/'completed';capture=completed/'capture';capture.mkdir(parents=True)
        frozen_suite=root/'frozen-suite.json';frozen_suite.write_text('{"initial":"same"}')
        previous=dict(native_code='/native',assets='/assets',opening_mm=83.,finger_envelope_sha256='a'*64,initial_suite=str(frozen_suite))
        (completed/'pipeline-config.json').write_text(json.dumps(previous))
        for name,value in [('final.json',{'status':'observed'}),('profile.json',{}),('reset.json',{}),('rgbd-manifest.json',{'frames':[{'path':f'{i}.npz'} for i in range(6)]})]:
            (capture/name).write_text(json.dumps(value))
        for i in range(6):(capture/f'{i}.npz').write_bytes(bytes([i]))
        resumed=dict(previous,observation_input_root=str(capture),remote_run_root=str(root/'new'))
        required=[completed/'pipeline-config.json',frozen_suite,*capture.iterdir()]
        proof={'provenance':{'input_sha256':{str(p):digest(p.read_bytes()) for p in required}}}
        assert verify_observation(resumed,proof)['reused'] is True
        detection=completed/'detection';detection.mkdir()
        (detection/'summary.json').write_text(json.dumps({'manifest_sha256':'sha256:'+digest((capture/'rgbd-manifest.json').read_bytes()),'adapter_sha256':'sha256:'+'a'*64,'node_config_sha256':'sha256:'+'b'*64}))
        (detection/'observations.jsonl').write_text('{}\n')
        resumed.update(detection_input_root=str(detection),detection_script='/adapter',detection_node_config='/node')
        proof['provenance']['input_sha256'].update({'/adapter':'a'*64,'/node':'b'*64,**{str(p):digest(p.read_bytes()) for p in detection.iterdir()}})
        assert verify_detection(resumed,proof)['reused'] is True
        (detection/'observations.jsonl').write_text('changed')
        rejects(lambda value:verify_detection(value,proof),resumed)
        rejects(lambda value:verify_observation(value,{}),resumed)
        rejects(lambda value:verify_observation(value,proof),dict(resumed,opening_mm=80.))
        (capture/'0.npz').write_bytes(b'changed')
        rejects(lambda value:verify_observation(value,proof),resumed)
        (capture/'failure.json').write_text('{}')
        rejects(lambda value:verify_observation(value,proof),resumed)
        source_command = shlex.split(stages["replay-source"]["argv"][-1])[-1]
        target_command = shlex.split(stages["replay-target"]["argv"][-1])[-1]
        assert "--suite " + config["initial_suite"] in source_command
        assert "--suite " + remote + "/capture/suite.json" not in source_command
        assert "--suite " + remote + "/detection-bridge/suite.json" in target_command
        assert "--record-raw" in source_command
        assert "export-playback" in stages
        assert sum(s["output"] is not None and s["output"].startswith("playback/") for s in stages.values()) == 7
        config_path.write_text(json.dumps({**config, "record_raw": False}))
        nonraw = run(config_path, root / "nonraw-dry", dry_run=True)["stages"]
        assert not any(s["name"] == "export-playback" for s in nonraw)
        assert all("--whitebox-fast" not in s["argv"][-1] for s in nonraw if s["group"] == "replay")
        config_path.write_text(json.dumps(config))
        assert "ROBOT_SIM_PROFILE_CAPTURE=1" in source_command
        assert remote + "/cache/torch-extensions" in stages["detect"]["argv"][-1]
        assert "raw capture requires CPU libx265" in stages["preflight-dsw-scene"]["argv"][-1]
        shortened, seconds = bounded_command(stages["replay-target"], 200)
        assert seconds == 155 and shlex.split(shortened[-1])[:4] == ["timeout", "-k", "30", "155"]
        assert bounded_command(stages["replay-target"], 900)[1] == 600
        try:
            bounded_command(stages["replay-target"], 45)
        except TimeoutError:
            pass
        else:
            raise AssertionError("a command started without shutdown time remaining")

        # Full raw capture measured 883s: only an explicit contract may raise
        # the replay cap, and elapsed earlier stages still reduce its budget.
        extended = deepcopy(contract)
        extended["resources"].update(max_wall_time_minutes=40, max_replay_time_seconds=1800)
        contract_path.write_text(json.dumps(extended))
        raw_stages = run(config_path, root / "raw-dry", dry_run=True)["stages"]
        for stage in raw_stages:
            expected = 1800 if stage["group"] == "replay" else 600
            assert bounded_command(stage, 2400)[1] == expected
            assert shlex.split(stage["argv"][-1])[:4] == ["timeout", "-k", "30", str(expected)]
            if stage["group"] == "replay":
                assert bounded_command(stage, 1000)[1] == 955
        for invalid in (None, True, 0, -1, 1800.5, "1800", 2400, float("inf")):
            extended["resources"]["max_replay_time_seconds"] = invalid
            contract_path.write_text(json.dumps(extended))
            rejects(load_config, config_path)
        contract_path.write_text(json.dumps(contract))

        # Configuration cannot widen the GPU, path or shell-command boundary.
        for change in ({"shell": "touch /tmp/no"}, {"native_code": "/remote/input/../escape"},
                       {"opening_mm": True}, {"opening_mm": 90}, {"record_raw": "true"},
                       {"remote_run_root": "/mnt/data/feiyu.zhou/runs"}):
            config_path.write_text(json.dumps({**config, **change}))
            rejects(load_config, config_path)
        config_path.write_text(json.dumps(config))
        bad = deepcopy(contract)
        bad["resources"]["lanes"][PIPELINE_LANES["dsw-scene"]["instance_id"]] = [0, 1]
        contract_path.write_text(json.dumps(bad))
        rejects(load_config, config_path)
        contract_path.write_text(json.dumps(contract))

        fixtures = root / "fixtures"
        fixtures.mkdir()
        for camera in PREVIEW_CAMERAS:
            PreviewPublisher(camera_frame_path(fixtures, camera)).publish(camera.encode(), {'camera': camera})
        source, plan, executions = fixture_messages()
        source.update(adapter="foundationpose", observation_mode="offline_replay")
        # These are transport fixtures only, not detector or raw validation results.
        for receipt in executions:
            actual = deepcopy(source["scene"]["episode_scene"])
            if receipt["side"] == "simulated-real":
                actual["environment_from_cup"]["position"][0] += .01
            receipt.update(actual_episode_scene=actual, actual_episode_digest=_digest(actual),
                           detected_scene_digest=source["scene"]["scene_digest"],
                           recording={"capture_status": "complete", "validation": "passed"})
        for name, value in (("source.json", source), ("plan.json", plan),
                            ("execution-source.json", executions[0]), ("execution-target.json", executions[1])):
            (fixtures / name).write_text(json.dumps(value))
        for name in ("source.png", "target.png"):
            (fixtures / name).write_bytes(b"\x89PNG\r\n\x1a\nfixture")
        playback_files = {}
        for name in PLAYBACK_FILES:
            (fixtures / name).write_bytes(b"browser-video-fixture")
            playback_files[name] = {"sha256": sha256(fixtures / name), "size_bytes": (fixtures / name).stat().st_size,
                                    "probe": {"codec_name": "h264", "pix_fmt": "yuv420p", "width": 640, "height": 480,
                                              "r_frame_rate": "30/1", "nb_read_frames": "2"}}
        playback_manifest = {"schema_version": "workbench_playback.v1", "run_id": Path(remote).name,
                             "clock": "simulation", "timeline_digest": plan["timeline_digest"],
                             "camera_frames": 2, "files": playback_files}
        (fixtures / "manifest.json").write_text(json.dumps(playback_manifest))
        fake_bin = root / "bin"
        fake_bin.mkdir()
        fake = fake_bin / "ssh"
        fake.write_text(f"#!{sys.executable}\n" + '''import json,os,pathlib,shlex,sys,time
command=shlex.split(sys.argv[-1])[-1]
with open(os.environ['FAKE_CALLS'],'a') as log:
 log.write(json.dumps({'command':command,'time':time.time()})+'\\n')
if 'workbench_replay_receipt' in command:
 if os.environ.get('FAKE_FAIL')=='replay-missing':
  print('{}')
 elif os.environ.get('FAKE_FAIL')=='replay-zero':
  print(json.dumps({'failure':{'error':'cup_machine_forbidden_contact at tick 344'}}))
 else:
  print(json.dumps({'final':{'status':'replay-complete','ticks':2},'execution':{'status':'completed','tick_count':2,'timeline_digest':'fixture'}}))
 sys.exit(0)
if '--result-timeout-s' in command and os.environ.get('FAKE_FAIL')=='detect':
 print('fixture detector failure'); sys.exit(23)
if '--mode replay' in command:
 time.sleep(.2)
if '/live_preview.py stream' in command:
 argv=shlex.split(command)
 name=pathlib.Path(argv[argv.index('--path')+1]).name
 # The receiver accepts the atomic frame magic, with or without stdout framing.
 sys.stdout.buffer.write((pathlib.Path(os.environ['FAKE_FIXTURES'])/name).read_bytes())
 sys.exit(0)
if 'sys.stdout.buffer.write' in command:
 remote=shlex.split(command)[-1]
 name=('execution-source.json' if '/replay-source/execution.json' in remote else
       'execution-target.json' if '/replay-target/execution.json' in remote else
       'source.png' if '/replay-source/final-camera.png' in remote else
       'target.png' if '/replay-target/final-camera.png' in remote else pathlib.Path(remote).name)
 sys.stdout.buffer.write((pathlib.Path(os.environ['FAKE_FIXTURES'])/name).read_bytes())
''')
        fake.chmod(0o700)
        calls = root / "calls.jsonl"
        environ = {"PATH": str(fake_bin) + os.pathsep + os.environ["PATH"],
                   "FAKE_CALLS": str(calls), "FAKE_FIXTURES": str(fixtures)}
        with patch.dict(os.environ, environ):
            result = run(config_path, root / "passed")
        assert result["status"] == "replay-complete" and result["synchronized"] is False, result
        assert result["pipeline"]["timing"]["status"] == "unverified"
        assert result["pipeline"]["playback"]["timeline_digest"] == plan["timeline_digest"]
        assert all((root / "passed/playback" / name).is_file() for name in ("manifest.json", *PLAYBACK_FILES))
        for side in ('source', 'target'):
            for camera in PREVIEW_CAMERAS:
                frame = read_frame(camera_frame_path(root / 'passed' / ('replay-'+side), camera))
                assert frame is not None and frame.metadata['camera'] == camera
                assert frame.payload == camera.encode()
        playback_path = root / "passed/playback" / PLAYBACK_FILES[0]
        original_video = playback_path.read_bytes()
        playback_path.write_bytes(b"corrupted-download")
        rejects(lambda output: verify(output, True), root / "passed")
        playback_path.write_bytes(original_video)
        manifest_path = root / "passed/playback/manifest.json"
        manifest_path.write_text(json.dumps({**playback_manifest, "timeline_digest": "other"}))
        rejects(lambda output: verify(output, True), root / "passed")
        manifest_path.write_text(json.dumps(playback_manifest))
        assert set(result["artifacts"]) == {"source.json", "plan.json", "execution-source.json",
                                           "execution-target.json", "source.png", "target.png"}
        rows = [json.loads(line) for line in calls.read_text().splitlines()]
        replay_starts = [row["time"] for row in rows if "--mode replay" in row["command"]]
        assert len(replay_starts) == 2 and abs(replay_starts[0] - replay_starts[1]) < .2
        assert json.loads((root / "passed/progress.json").read_text())["status"] == "replay-complete"
        try:
            run(config_path, root / "passed")
        except FileExistsError:
            pass
        else:
            raise AssertionError("existing output was overwritten")

        # Isaac may exit zero during close despite a native failure. Do not export video.
        zero_calls = root / "zero-calls.jsonl"
        with patch.dict(os.environ, {**environ, "FAKE_CALLS": str(zero_calls), "FAKE_FAIL": "replay-zero"}):
            zero = run(config_path, root / "zero-failure")
        assert zero["status"] == "failed" and "cup_machine_forbidden_contact" in zero["error"]
        zero_state = json.loads((root / "zero-failure/run.json").read_text())
        assert all(r["exit_code"] is None for r in zero_state["receipts"] if r["stage"].startswith("replay-"))
        assert json.loads((root / "zero-failure/progress.json").read_text())["stage"] == "replay"
        assert "export_playback.py" not in zero_calls.read_text()
        with patch.dict(os.environ, {**environ, "FAKE_FAIL": "replay-missing"}):
            missing = run(config_path, root / "missing-receipt")
        assert missing["status"] == "failed" and "missing or incomplete native replay" in missing["error"]
        failed_calls = root / "failed-calls.jsonl"
        with patch.dict(os.environ, {**environ, "FAKE_CALLS": str(failed_calls), "FAKE_FAIL": "detect"}):
            result = run(config_path, root / "failed")
        assert result["status"] == "failed" and "detect" in result["error"]
        assert "logs/detect.log" in result["error"] and "exit=23" in result["error"]
        state = json.loads((root / "failed/run.json").read_text())
        assert state["receipts"][-1]["exit_code"] == 23
        assert "--mode place" not in failed_calls.read_text()
        assert not (root / "failed/source.json").exists()
        assert "fixture detector failure" in (root / "failed/logs/detect.log").read_text()
        # A corrupted downloaded receipt is retained for diagnosis, never promoted as success.
        (fixtures / "execution-target.json").write_text(json.dumps({**executions[1], "tick_count": 1}))
        with patch.dict(os.environ, environ):
            result = run(config_path, root / "invalid")
        assert result["status"] == "failed" and not (root / "invalid/source.json").exists()
        assert (root / "invalid/.artifacts/execution-target.json").exists()
        (fixtures / "execution-target.json").write_text(json.dumps(executions[0]))
        with patch.dict(os.environ, environ):
            result = run(config_path, root / "duplicate-lane")
        assert result["status"] == "failed" and "wrong lane" in result["error"]
    print("Pipeline dry-run, fixed GPU/SSH boundaries, concurrent replay and failure retention: PASS")


if __name__ == "__main__":
    main()
