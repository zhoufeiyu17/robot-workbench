"""CPU-only checks for replay identity, native raw cadence and recorder lifecycle."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
from concurrent.futures import ThreadPoolExecutor
import time
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

from integration import _digest
from isaac_lane import (bind_effective_gripper_identity, close_failed_recording, enable_forced_diagnostic_replay,
                        make_software_encoder, raw_digest, raw_job,
                        raw_recording_summary, recorded_step, replay_telemetry, validate_record_options, validate_replay_inputs,
                        validate_physical_identity, configure_whitebox_fast_scene,
                        preview_sensors, publish_previews, preview_stats,
                        replay_handshake, replay_tick_sync, replay_provenance, replay_scheduling,
                        configure_observation_scene, validate_observation_outputs, confirm_tick, wait_for, publish_failure, close_runtime)
from test_app import fixture_messages
from test_harness import rejects


def main():
    # A preview uses only existing camera buffers and leaves the scene cadence alone.
    class RGBBuffer:
        def __init__(self, camera):
            self.camera = camera
        def __getitem__(self, key):
            assert key == (0, Ellipsis, slice(None, 3))
            return self.camera.encode()
    camera_names = ('base_0', 'left_wrist_0', 'right_wrist_0')
    existing = {'d405_'+name: SimpleNamespace(data=SimpleNamespace(output={'rgb': RGBBuffer(name)}))
                for name in camera_names}
    sensors = preview_sensors(existing)
    assert tuple(sensors) == camera_names
    with TemporaryDirectory() as directory:
        from live_preview import PreviewPublisher, camera_frame_path, read_frame
        publishers = {name: PreviewPublisher(camera_frame_path(directory, name)) for name in sensors}
        with patch('isaac_lane.publish_rgb', side_effect=lambda publisher, rgb, meta: publisher.publish(rgb, meta)),patch('isaac_lane.calibrated_camera_rgb',side_effect=lambda env,name:(name+'-calibrated').encode()) as projection:
            publish_previews(publishers, sensors, {'lane': 'dsw-scene', 'mode': 'replay', 'tick': 12},env=existing)
        assert projection.call_count==3
        for name in camera_names:
            frame = read_frame(camera_frame_path(directory, name))
            assert frame.payload == (name+'-calibrated').encode()
            assert frame.metadata['camera'] == name and frame.metadata['sensor'] == 'd405_'+name
            assert frame.metadata['tick'] == 12 and frame.metadata['image_domain'] == 'calibrated_color'
        stats = preview_stats(publishers)
        assert stats['published_frames'] == 1 and set(stats['cameras']) == set(camera_names)
    whitebox = preview_sensors({'workspace_camera': existing['d405_base_0']}, whitebox_fast=True)
    assert tuple(whitebox) == ('base_0',)
    # RGB-D must share the calibrated ray map and avoid depth interpolation
    # across object boundaries; this stays independent of Torch/Isaac imports.
    import sys
    from isaac_lane import calibrated_camera_depth,calibrated_camera_k
    class Depth:
        ndim=3;shape=(1,640,800)
        def unsqueeze(self,axis):assert axis==1;return self
        def float(self):return self
    grid=SimpleNamespace(expand=lambda *shape:'same-rgb-rays')
    depth=Depth();env=SimpleNamespace(scene={'d405_base_0':SimpleNamespace(data=SimpleNamespace(output={'distance_to_image_plane':depth}))},
        _d405_grid_cache={'base_0':grid},cfg=SimpleNamespace(d405_rig={'device_calibrations':{'base_0':{'intrinsics':dict(fx=392.,fy=391.,cx=315.,cy=234.)}}}))
    class Sample:
        def __getitem__(self,key):assert key==(0,0);return 'depth640x480'
    sampler=Mock(return_value=Sample())
    with patch.dict(sys.modules,{'torch.nn.functional':SimpleNamespace(grid_sample=sampler)}):
        assert calibrated_camera_depth(env,'base_0')=='depth640x480'
    assert sampler.call_args.args==(depth,'same-rgb-rays') and sampler.call_args.kwargs==dict(align_corners=True,mode='nearest')
    assert calibrated_camera_k(env,'base_0')==[[392.,0.,315.],[0.,391.,234.],[0.,0.,1.]]

    camera = SimpleNamespace(width=640, height=480, data_types=["rgb", "distance_to_image_plane"], update_period=0.0)
    scene = SimpleNamespace(workspace_camera=camera, room=object(), d435_model=object(), handoff_stand=object(),
                            dome_light=object(), daylight_key=object())
    configure_whitebox_fast_scene(SimpleNamespace(scene=scene))
    assert camera.width == 640 and camera.height == 480 and camera.data_types == ["rgb"] and scene.room is None
    assert scene.dome_light is not None and scene.daylight_key is not None
    assert camera.update_period == 1 / 30
    rig = {"cameras": [{"device_calibration_ref": name} for name in ("base_0", "left_wrist_0", "right_wrist_0")]}
    cfg = SimpleNamespace(scene=SimpleNamespace(d405_base_0=camera, d405_left_wrist_0=SimpleNamespace(data_types=["rgb"]),
                                               d405_right_wrist_0=SimpleNamespace(data_types=["rgb"])), d405_rig=rig,
                          observations=SimpleNamespace(rgb=SimpleNamespace(base_0=object(),
                              left_wrist_0=object(), right_wrist_0=object())))
    configure_observation_scene(cfg)
    assert cfg.scene.d405_left_wrist_0 is not None and cfg.scene.d405_right_wrist_0 is not None
    assert camera.data_types == ["rgb", "distance_to_image_plane"]
    assert cfg.scene.d405_left_wrist_0.data_types == cfg.scene.d405_right_wrist_0.data_types == camera.data_types
    assert cfg.observations.rgb.base_0 is not None
    assert cfg.observations.rgb.left_wrist_0 is not None and cfg.observations.rgb.right_wrist_0 is not None
    assert cfg.d405_rig is rig and len(rig["cameras"]) == 3
    outputs = {name: SimpleNamespace(data=SimpleNamespace(output={key: object() for key in sensor.data_types}))
               for name, sensor in vars(cfg.scene).items()}
    validate_observation_outputs(outputs)
    del outputs["d405_left_wrist_0"].data.output["distance_to_image_plane"]
    try:
        validate_observation_outputs(outputs)
    except ValueError as error:
        assert "d405_left_wrist_0" in str(error) and "distance_to_image_plane" in str(error)
    else:
        raise AssertionError("missing wrist depth must fail before observation capture")
    with TemporaryDirectory() as directory:
        barrier = Path(directory)
        progress = [0, 0]
        def lane(index, side):
            for tick in range(5):
                time.sleep(.003 * index)
                progress[index] += 1
                assert abs(progress[0] - progress[1]) <= 1
                confirm_tick(barrier, side, tick)
        with ThreadPoolExecutor(max_workers=2) as pool:
            jobs = [pool.submit(lane, i, side) for i, side in enumerate(("simulated-real", "dsw-scene"))]
            for job in jobs:
                job.result()
        assert progress == [5, 5] and len(list(barrier.glob('*.json'))) == 10
        (barrier / "dsw-scene-failed.json").write_text('{}')
        # Even an already-published completion cannot hide a peer failure.
        try:
            wait_for(barrier / "dsw-scene-4-done.json", .01)
            raise AssertionError("failure did not stop barrier")
        except RuntimeError:
            pass

    with TemporaryDirectory() as directory:
        root = Path(directory)
        args = SimpleNamespace(output=root, barrier=root, mode="replay", side="simulated-real", run_id="test")
        publish_failure(args, RuntimeError("cup_machine_forbidden_contact"))
        try:
            wait_for(root / "simulated-real-344-done.json", .01)
        except RuntimeError as error:
            assert "cup_machine_forbidden_contact" in str(error)
        else:
            raise AssertionError("peer did not immediately see root failure")
        class ClosingApp:
            def close(self):
                raise SystemExit(0)
        try:
            try:
                raise RuntimeError("original collision")
            finally:
                close_runtime(None, ClosingApp())
        except RuntimeError as error:
            assert str(error) == "original collision"
        else:
            raise AssertionError("shutdown hid native failure")

    class FakeEvaluator:
        def __init__(self):
            self.event = None
            self._current_physics_step = 12

        def fail(self, reason, *, status="completed"):
            self.event = SimpleNamespace(reason=reason, run_status=status)
            return self.event

        def observe(self, *args, **kwargs):
            self.event = SimpleNamespace(reason="success", run_status="completed")
            return self.event

    evaluator = FakeEvaluator()
    runtime = SimpleNamespace(evaluator=evaluator)
    forced = enable_forced_diagnostic_replay(runtime)
    assert evaluator.fail("cup_machine_forbidden_contact") is None and evaluator.event is None
    assert forced == [{"reason": "cup_machine_forbidden_contact", "status": "completed", "physics_step": 12}]
    assert evaluator.fail("backend_failure").reason == "backend_failure"
    evaluator.event = None
    assert evaluator.observe(None) is None and evaluator.event is None

    from isaac_lane import validate_reset_grippers
    from isaac_lane import runtime_profile_snapshot
    rows={'control_step':{'calls':1,'inclusive_seconds':.1,'exclusive_seconds':.02}}
    probe=runtime_profile_snapshot(SimpleNamespace(profile=SimpleNamespace(rows=rows)),16)
    rows['control_step']['calls']=2
    assert probe['completed_ticks']==16 and probe['rows']['control_step']['calls']==1
    assert runtime_profile_snapshot(None,16) is None
    from isaac_lane import incidental_claw_contacts
    root='/World/envs/env_0';body=root+'/Robot/left_link7'
    acceptance=SimpleNamespace(open_position_m=.04,open_tolerance_m=.001,finger_force_max_n=100.,residual_force_max_n=2.,finger_contact_speed_limit=lambda grasp:.16)
    sample=SimpleNamespace(jaw_positions=[.04,.04],finger_cup_relative_speed_m_s=.01)
    metal=SimpleNamespace(actors=(body,root+'/Cup'),shapes=(body+'/collisions/metal',root+'/Cup/Rim'),impulse=(.005,0.,0.),separation_m=-.002)
    pad=SimpleNamespace(**{**vars(metal),'shapes':(body+'/RealFingerRubberPad',root+'/Cup/Body')})
    filtered,allowed=incidental_claw_contacts([metal,pad],sample,acceptance,root,.01,grasp_confirmed=False,opening=True)
    assert filtered==(pad,) and allowed==[metal]
    deep=SimpleNamespace(**{**vars(metal),'separation_m':-.0031})
    assert incidental_claw_contacts([deep],sample,acceptance,root,.01,grasp_confirmed=False,opening=True)[0]==(deep,)
    strong=SimpleNamespace(**{**vars(metal),'impulse':(.03,0.,0.)})
    assert incidental_claw_contacts([strong],sample,acceptance,root,.01,grasp_confirmed=False,opening=True)[0]==(strong,)
    from isaac_lane import publish_raw_spool
    with TemporaryDirectory() as folder:
        base=Path(folder);spool=base/'spool';record=spool/'cache/episode';record.mkdir(parents=True)
        (record/'state.json').write_text('{"frames":17}')
        (base/'published').mkdir()
        cap=SimpleNamespace(_workbench_spool=spool,_workbench_publish=base/'published/raw',record=SimpleNamespace(root=record))
        published=publish_raw_spool(cap)
        assert published==base/'published/raw/cache/episode' and (published/'state.json').read_bytes()==(record/'state.json').read_bytes()
        cap._workbench_publish=base/'published/bad'
        with patch('isaac_lane.file_sha256',side_effect=['source','changed']):
            rejects(publish_raw_spool,cap)
        assert not cap._workbench_publish.exists()
    partial=SimpleNamespace(metadata={'initial_robot_state':{'left_gripper':71.2266,'right_gripper':.7642}})
    assert validate_reset_grippers({'left_gripper':71.3,'right_gripper':.8},partial,82.992)=={'left':71.2266,'right':.7642}
    rejects(lambda row:validate_reset_grippers(row,partial,82.992),{'left_gripper':72.,'right_gripper':.8})
    assert validate_reset_grippers({'left_gripper':82.992,'right_gripper':82.992},SimpleNamespace(metadata=None),82.992)=={'left':82.992,'right':82.992}
    runtime = SimpleNamespace(physical_identity={"authored_open_position_m": .022})
    limits = {name: [0., .0415] for name in ("left_joint7", "left_joint8", "right_joint7", "right_joint8")}
    bind_effective_gripper_identity(runtime, .0415, limits)
    assert runtime.physical_identity == {"authored_open_position_m": .022,
        "effective_gripper_opening_mm": 83.0, "effective_gripper_open_position_m": .0415,
        "effective_gripper_joint_limits_m": limits,
        "gripper_limit_source": "PhysX readback; development displacement range"}
    assert runtime.physical_sha256 == hashlib.sha256(json.dumps(
        runtime.physical_identity, sort_keys=True, allow_nan=False).encode()).hexdigest()
    native = {"native_identity": {"physical_configuration": deepcopy(runtime.physical_identity),
                                 "physical_configuration_sha256": runtime.physical_sha256}}
    validate_physical_identity(runtime, native)
    telemetry_runtime = SimpleNamespace(
        loop=SimpleNamespace(physics_steps=1),
        read=lambda _: (SimpleNamespace(cup_velocity=(0.01, 0.0, 0.0)), None),
        evaluator=SimpleNamespace(last_audit={"active_finger_contact_count": 2, "contact_count": 3,
                                              "support_force_n": 1.2, "finger_forces_n": {"left": .4, "right": .5}},
                                  facts={"support_acquired": True}),
    )
    row = replay_telemetry(telemetry_runtime, "open", {"tick": 770, "time_s": 25.667},
                           {"left_gripper": 83.0, "right_gripper": 83.0},
                           {"event": {"reason": "grasp_lost"}, "actual_substeps": 2,
                            "requested_substeps": 8, "terminated": True, "action_applied": True},
                           {"left_gripper": 83.0, "right_gripper": 83.0})
    assert row["grasp_state"] == "lost" and row["finger_contact_count"] == 2 and row["tick"] == 770
    runtime.physical_identity["gravity"] = (0., 0., -9.81)
    bind_effective_gripper_identity(runtime, .0415, limits)
    native = {"native_identity": {"physical_configuration": json.loads(json.dumps(runtime.physical_identity)),
                                 "physical_configuration_sha256": runtime.physical_sha256}}
    validate_physical_identity(runtime, native)  # Native tuples round-trip as JSON arrays.
    native["native_identity"]["physical_configuration"]["effective_gripper_joint_limits_m"]["left_joint8"][1] = .044
    rejects(lambda p: validate_physical_identity(runtime, p), native)
    source, plan, _ = fixture_messages()
    plan.update(control={"physics_hz": 240, "substeps": 8}, native_actions=[[0]*14]*2,
                native_substeps=[8, 4], motion_profile="continuous-baseline")
    suite = {"episodes": [deepcopy(source["scene"]["episode_scene"])]}
    with TemporaryDirectory(prefix="isaac-raw-protocol-") as folder:
        root = Path(folder)
        prompt = {"text": "将纸杯放在咖啡机出水口下", "language": "zh-CN", "version": "fixture-v1"}
        prompt_file = root / "prompt.json"
        prompt_file.write_text(json.dumps(prompt, ensure_ascii=False))
        args = SimpleNamespace(record_raw=False, mode="replay", raw_prompt_file=None, output=root,
                               assets=root / "assets", run_id="fixture-replay", side="dsw-scene",
                               instance="dsw-xs4hu2jkqj2igqaoul", gpu=0)
        assert validate_record_options(args) is None  # No recorder or prompt is needed by default.
        args.record_raw = True
        rejects(validate_record_options, args)
        args.raw_prompt_file = prompt_file
        assert validate_record_options(args) == prompt
        args.mode = "observe"
        rejects(validate_record_options, args)
        args.mode = "replay"
        projection = validate_replay_inputs(args, plan, source, suite)
        simulated_independent=SimpleNamespace(**{**vars(args),'independent_replay':True,'hardware_binding':None,'barrier':None})
        assert validate_record_options(simulated_independent)==prompt
        assert validate_replay_inputs(simulated_independent,plan,source,suite)==projection
        with patch('isaac_lane.write',side_effect=AssertionError('unexpected barrier write')), \
             patch('isaac_lane.wait_for',side_effect=AssertionError('unexpected peer wait')), \
             patch('isaac_lane.confirm_tick',side_effect=AssertionError('unexpected tick barrier')):
            replay_handshake(simulated_independent);replay_tick_sync(simulated_independent,0)

        assert projection["actual_episode_digest"] == _digest(suite["episodes"][0])
        physical_source = {k:deepcopy(v) for k,v in source.items() if k not in ('instance_id','gpu')}
        physical_source.update(schema_version='physical_collection_source.v1',lane='physical-real',
                               evidence_source='physical_rgbd',robot_id='fixture',adapter='foundationpose',
                               camera_serials=dict(head='h',left='l',right='r'))
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
                     camera_serials=physical_source['camera_serials'],source_digest=_digest(physical_source),
                     calibration_profile_digest=_digest(physical_source['calibration_profile']))
        binding_path=root/'hardware-binding.json';binding_path.write_text(json.dumps(binding))
        real_args=SimpleNamespace(**{**vars(args),'independent_replay':True,'hardware_binding':binding_path,'barrier':None})
        assert validate_record_options(real_args)==prompt
        dual_plan = deepcopy(plan)
        from test_joint_space import physical_fixture
        physical_fixture(plan, physical_source)
        assert validate_replay_inputs(real_args,plan,physical_source,suite)==projection
        assert replay_provenance(plan)['evidence_source']=='simulation'
        assert replay_scheduling(real_args)=='independent simulation replay'
        with patch('isaac_lane.write',side_effect=AssertionError('unexpected barrier write')), \
             patch('isaac_lane.wait_for',side_effect=AssertionError('unexpected peer wait')), \
             patch('isaac_lane.confirm_tick',side_effect=AssertionError('unexpected tick barrier')):
            replay_handshake(real_args)
            replay_tick_sync(real_args,0)
        rejects(lambda a:validate_replay_inputs(a,plan,source,suite),real_args)  # Binding, rather than independent timing, makes this physical-only.
        for change in ({'side':'simulated-real','instance':'dsw-nygxkzfd5znx4if0t2'},
                       {'force_diagnostic_replay':True},{'independent_replay':False},
                       {'hardware_binding':None},{'barrier':root},{'mode':'observe'}):
            bad_args=SimpleNamespace(**{**vars(real_args),**change})
            rejects(lambda a:validate_replay_inputs(a,plan,physical_source,suite),bad_args)
            if change!={'hardware_binding':None}:rejects(validate_record_options,bad_args)
        wrong=deepcopy(physical_source);wrong['robot_id']='other'
        rejects(lambda source:validate_replay_inputs(real_args,plan,source,suite),wrong)
        plan = dual_plan
        moved = deepcopy(suite)
        moved["episodes"][0]["environment_from_cup"]["position"][0] += .04
        rejects(lambda value: validate_replay_inputs(args, plan, source, value), moved)
        args.side, args.instance, args.gpu = "simulated-real", "dsw-nygxkzfd5znx4if0t2", 0
        observed = validate_replay_inputs(args, plan, source, moved)
        assert observed["actual_episode_digest"] != projection["actual_episode_digest"]
        assert observed["detected_scene_digest"] == source["scene"]["scene_digest"]
        assert set(observed["actual_episode_scene"]) == set(source["scene"]["episode_scene"])
        args.instance = "dsw-xs4hu2jkqj2igqaoul"
        rejects(lambda value: validate_replay_inputs(value, plan, source, suite), args)
        args.instance = "dsw-nygxkzfd5znx4if0t2"
        job = raw_job(args, plan, source, "fixture-episode", "fixture-task", prompt, {"runner.py": "fixture-sha"})
        assert job["encoder_options"]["encoder"] == "libx265"
        assert job["batch"]["prompt_sha256"] == hashlib.sha256(json.dumps(
            prompt, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()
        assert job["generation_plan"]["parameters_resolved"]["motion_profile"] == "continuous-baseline"
        for bad_steps in ([4, 8], [8, 0], [8], [True, 4]):
            rejects(lambda value: raw_job(args, value, source, "e", "t", prompt, {}), {**plan, "native_substeps": bad_steps})
        rejects(lambda value: raw_job(args, value, source, "e", "t", prompt, {}),
                {**plan, "control": {"physics_hz": 120, "substeps": 8}})

        # Exercise the exact VideoWriter ffmpeg hook without a video dependency or GPU.
        fake = root / "fake-ffmpeg"
        fake.write_text("#!/usr/bin/python3\nimport json,sys\nopen(sys.argv[-1],'w').write(json.dumps(sys.argv[1:]))\n")
        fake.chmod(0o700)
        work = root / "work"
        work.mkdir()
        with patch("isaac_lane.shutil.which", return_value=str(fake)):
            encoder = make_software_encoder(work)
        argv_file = root / "encoder-argv.json"
        subprocess.run([str(encoder), "-i", "pipe:0", "-c:v", "libx265", str(argv_file)], check=True)
        argv = json.loads(argv_file.read_text())
        assert argv[-5:] == ["-threads", "2", "-x265-params", "pools=2:frame-threads=1", str(argv_file)]
        assert "libx265" in argv and "hevc_nvenc" not in argv

        (root / "manifests").mkdir()
        (root / "camera").mkdir()
        manifest = {"schema_version": "sim_raw_dataset/v1", "capture_status": "complete", "outcome": "success",
                    "validation": {"status": "passed"}, "final_physics_step": 12}
        path = root / "manifests/episode.json"
        path.write_text(json.dumps(manifest))
        metadata = {d+"_rgb": {"frames": 2} for d in ("base_0", "left_wrist_0", "right_wrist_0")}
        (root / "camera/metadata.json").write_text(json.dumps(metadata))
        summary = raw_recording_summary(root, plan)
        assert summary["native_state_samples"] == 13 and summary["robot_samples_per_arm"] == 7
        assert set(summary["camera_frames"].values()) == {2}  # k=0,8; no invented frame at partial k=12.
        assert summary["manifest_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
        for field, value in (("capture_status", "recording"), ("outcome", "execution_failed"),
                             ("final_physics_step", 16), ("validation", {"status": "failed"})):
            path.write_text(json.dumps({**manifest, field: value}))
            rejects(lambda value: raw_recording_summary(root, value), plan)
        path.write_text(json.dumps(manifest))
        metadata["base_0_rgb"]["frames"] = 3
        (root / "camera/metadata.json").write_text(json.dumps(metadata))
        rejects(lambda value: raw_recording_summary(root, value), plan)

        events = []
        record = SimpleNamespace(active_command={}, finished=False, root=root,
                                 end_command=lambda: events.append("end"))
        capture = SimpleNamespace(record=record,
            before_command=lambda decision: events.append(("begin", decision.stage)),
            finish=lambda *items: events.append(("finish", items[0])))
        env = SimpleNamespace(step=lambda request: events.append("step") or "result")
        assert recorded_step(env, capture, "request", "lift", 1, "timeline") == "result"
        assert events == [("begin", "lift"), "step", "end"]
        assert record.active_command["source_kind"] == "frozen_timeline_replay"
        def broken(request):
            raise RuntimeError("physics failed")
        env.step = broken
        try:
            recorded_step(env, capture, "request", "lift", 1, "timeline")
        except RuntimeError:
            pass
        else:
            raise AssertionError("step failure was hidden")
        assert events[-1] == "end"
        assert close_failed_recording(capture, {}, RuntimeError("physics failed"))["capture_status"] == "failed"
        assert events[-1] == ("finish", "execution_failed")
        capture.finish = broken
        failure = close_failed_recording(capture, {}, "error")
        assert "finalization_error" in failure
        record.finished = True
        assert "finalization_error" not in close_failed_recording(capture, {}, "error")
    print("Isaac replay identity, software encoder, partial-tick raw counts and failure lifecycle: PASS")


if __name__ == "__main__":
    main()
