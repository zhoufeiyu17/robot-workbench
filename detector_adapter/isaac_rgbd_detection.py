#!/usr/bin/env python3
"""Replay independently captured Isaac RGB-D frames through the real perception runtime.

Manifest: frame_id, depth_convention='optical_z_m', camera_pose_convention=
'world_from_ros_optical', frames=[{path, source_stamp_ns, capture_sequence_id}].
Each NPZ contains rgb, depth_m, K, position, quaternion_wxyz. The latter two
describe ONLY the camera extrinsic. Object ground truth is never an input.
This is offline replay, not a measurement of live transport latency.
"""
from __future__ import annotations

import time

_SCRIPT_STARTED_S = time.perf_counter()

import argparse
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

runtime_root = Path(os.environ.get('ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT',
                                  str(Path(__file__).resolve().parents[1])))
if not runtime_root.is_absolute() or not all((runtime_root/p).is_dir() for p in ('scripts','ros2/online_detection_ros')):
    raise ValueError('explicit existing detector runtime root required')
import isaac_detection_service as _service_adapter
sys.path[:0] = [str(runtime_root/'scripts'), str(runtime_root/'ros2/online_detection_ros')]

from run_online_detection_episode import current_pose, render, sha256, source_identity, write_json


def timing_mark(timing, previous_s, stage, **details):
    now = time.perf_counter()
    row = dict(stage=stage, duration_s=now - previous_s,
               elapsed_s=now - _SCRIPT_STARTED_S, unix_s=time.time(), **details)
    timing['segments'].append(row)
    callback = timing.get('_callback')
    if callback is not None:
        callback(row)
    print(json.dumps({'detection_timing': row}, allow_nan=False), flush=True)
    return now


def load_manifest(path):
    path = path.resolve(strict=True)
    manifest = json.loads(path.read_text())
    if (manifest.get('depth_convention') != 'optical_z_m'
            or manifest.get('camera_pose_convention') != 'world_from_ros_optical'
            or not isinstance(manifest.get('frame_id'), str) or not manifest['frame_id'].strip()):
        raise ValueError('explicit optical Z depth and world-from-ROS-optical camera convention required')
    rows = manifest.get('frames')
    if not isinstance(rows, list) or not rows:
        raise ValueError('frames must be nonempty')
    previous = (0, -1)
    seen = set()
    for row in rows:
        stamp, sequence = row['source_stamp_ns'], row['capture_sequence_id']
        if (type(stamp) is not int or type(sequence) is not int
                or stamp <= previous[0] or sequence <= previous[1]):
            raise ValueError('capture stamps and sequences must strictly increase')
        relative = Path(row['path'])
        frame = (path.parent / relative).resolve(strict=True)
        if relative.is_absolute() or not frame.is_relative_to(path.parent) or not frame.is_file() or frame in seen:
            raise ValueError('each frame must be a distinct file inside the manifest directory')
        seen.add(frame)
        previous = stamp, sequence
    if rows[-1]['source_stamp_ns'] - rows[0]['source_stamp_ns'] < 5_000_000_000:
        raise ValueError('capture window must span at least 5 seconds; timestamps must come from capture')
    return manifest


def read_frame(path):
    with np.load(path, allow_pickle=False) as data:
        rgb, depth, k, position, q = (data[name] for name in ('rgb', 'depth_m', 'K', 'position', 'quaternion_wxyz'))
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) <= 0:
        raise ValueError('rgb must be HxWx3 uint8')
    if depth.dtype != np.float32 or depth.shape != rgb.shape[:2] or not np.any(np.isfinite(depth) & (depth > 0)):
        raise ValueError('depth_m must be aligned float32 optical Z in meters with valid samples')
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1]) > 1e-8 or abs(k[1, 0]) > 1e-8):
        raise ValueError('K must be a finite zero-skew pinhole matrix')
    if position.shape != (3,) or not np.isfinite(position).all() or q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-5:
        raise ValueError('camera pose must contain finite position and unit quaternion_wxyz')
    return rgb, depth, k, position, q


def world_pose(position, camera_position, camera_q, quaternion_xyzw=None):
    w = camera_q[0]
    axis = np.asarray(camera_q[1:])
    point = np.asarray(position)
    rotated = point + 2 * np.cross(axis, np.cross(axis, point) + w * point)
    result = {'xyz_m': (rotated + camera_position).tolist()}
    if quaternion_xyzw is not None:
        other = np.asarray(quaternion_xyzw[:3])
        scalar = quaternion_xyzw[3]
        vector = w * other + scalar * axis + np.cross(axis, other)
        composed = np.r_[vector, w * scalar - np.dot(axis, other)]
        result['quaternion_xyzw'] = (composed / np.linalg.norm(composed)).tolist()
    return result


def extract(batch, camera_position, camera_q, canonical_from_mesh):
    from online_detection_ros_node.pose.foundationpose import _rotation_to_quaternion_xyzw
    tracks = []
    for item in batch.instances:
        if item.lifecycle.value != 'ACTIVE' or item.object_type not in ('coffee_machine', 'cup'):
            continue
        pose = item.pose
        if (current_pose(item, batch) and item.asset_match.identity_valid
                and pose.provider_id == 'foundationpose'):
            # Cup yaw is intentionally absent from the output.
            orientation = pose.quaternion_xyzw if item.object_type == 'coffee_machine' else None
            if orientation is not None and pose.orientation_ambiguous:
                continue
            provenance = dict(provider_id=pose.provider_id, provider_revision=pose.provider_revision,
                              model_revision=pose.model_revision, asset=asdict(pose.pose_asset))
            position = pose.position_xyz
            transform = np.asarray(canonical_from_mesh[pose.pose_asset]).reshape(4, 4)
            camera_from_canonical = np.eye(4)
            q = pose.quaternion_xyzw
            camera_from_canonical[:3, :3] = np.array([world_pose(axis, [0, 0, 0],
                [q[3], *q[:3]])['xyz_m'] for axis in np.eye(3)]).T
            camera_from_canonical[:3, 3] = position
            camera_from_mesh = camera_from_canonical @ transform
            mesh_orientation = _rotation_to_quaternion_xyzw(camera_from_mesh[:3, :3]) if orientation is not None else None
            provenance.update(output_frame='simulation_mesh_root',
                              canonical_from_simulation_mesh_root=transform.ravel().tolist(),
                              transform_source='verified ObjectAssetPack.mesh_to_canonical')
        else:
            continue
        tracks.append(dict(track_id=item.track_id, object_type=item.object_type,
                           xyz_m=list(position), pose_current=True, provenance=provenance,
                           **({'quaternion_xyzw': list(orientation)} if orientation is not None else {}),
                           canonical_world=world_pose(position, camera_position, camera_q, orientation),
                           world=world_pose(camera_from_mesh[:3, 3], camera_position, camera_q, mesh_orientation)))
    ready = sorted(t['object_type'] for t in tracks) == ['coffee_machine', 'cup']
    return dict(mode='offline_replay', source_stamp_ns=batch.source_stamp_ns, frame_id=batch.frame_id,
                source_sequence_id=batch.source_sequence_id, tracking_session_id=batch.tracking_session_id,
                runtime_config_id=batch.runtime_config_id, provider_status=batch.provider_run_status.value,
                pair_valid=ready, tracks=tracks)


def continuous_pair_window(observations):
    current, previous_pair = [], None
    for observation in observations:
        pair = (observation['tracking_session_id'], tuple(sorted(
            (track['object_type'], track['track_id'], json.dumps(track['provenance']['asset'], sort_keys=True))
            for track in observation['tracks']))) if observation['pair_valid'] else None
        if pair is None or pair != previous_pair:
            current = []
        if pair is not None:
            current.append(observation['source_stamp_ns'])
        previous_pair = pair
    return current


def initialize_detection(args, summary, timing, checkpoint, expected_frame_id=None):
    if sys.platform != 'linux':
        raise RuntimeError('inference is restricted to the authorized Linux CUDA target')
    from online_detection_ros_node.configuration.startup import load_startup_catalog
    from online_detection_ros_node.orchestration.application import prepare_application
    from online_detection_ros_node.pose.bootstrap import build_foundationpose_pose_stage
    from online_detection_ros_node.pose.configuration import compile_foundationpose_assets
    from online_detection_ros_node.pose.runtime_pack import compile_foundationpose_runtime_pack, validate_current_foundationpose_runtime
    from online_detection_ros_node.runtime.runtime_exec import _startup_from_node_config, _configure_pose_environment, _pose_dependency_pack
    from online_detection_ros_node.sensor.depth_source import ImageSample, CameraInfoSample

    visible = os.environ.get('CUDA_VISIBLE_DEVICES', '').strip()
    if not visible or ',' in visible or visible == '-1' or visible.split() != [visible]:
        raise ValueError('CUDA_VISIBLE_DEVICES must explicitly select one GPU')
    import torch
    if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise RuntimeError('exactly one visible CUDA device required')
    checkpoint = timing_mark(timing, checkpoint, 'runtime_imports_and_cuda')
    startup = _startup_from_node_config(args.node_config)
    if startup is None or startup.runtime_mode.value != 'DEVELOPMENT':
        raise ValueError('pinned DEVELOPMENT release required')
    startup = replace(startup, verify_development_optional_artifacts=True)
    catalog = load_startup_catalog(startup)
    if expected_frame_id is not None and expected_frame_id != catalog.runtime_config.sensor_profile.optical_frame:
        raise ValueError('manifest optical frame differs from release')
    canonical_from_mesh = {asset.binding.asset: asset.binding.mesh_to_canonical
                           for asset in compile_foundationpose_assets(catalog)}
    pack = compile_foundationpose_runtime_pack(catalog)
    if pack is None:
        raise ValueError('FoundationPose-enabled release required')
    validate_current_foundationpose_runtime(pack)
    checkpoint = timing_mark(timing, checkpoint, 'catalog_assets_runtime_validation')
    (args.run_dir / 'runtime_cache').mkdir()
    dependency_site = None
    if os.environ.get('ONLINE_DETECTION_POSE_DEPENDENCY_ROOT') or os.environ.get('ONLINE_DETECTION_POSE_DEPENDENCY_DIGEST'):
        dependency, dependency_site = _pose_dependency_pack(startup.runtime_mode)
        summary['dependency_pack_digest'] = dependency.manifest_digest
    _configure_pose_environment(pack, args.run_dir, dependency_site)
    checkpoint = timing_mark(timing, checkpoint, 'configure_environment')
    from arx_render_batches import install
    summary.update(render_batching=install(pack.root, batchsize=args.render_batch_size),
                   node_config_sha256=sha256(args.node_config), release_manifest_digest=startup.release_manifest_digest,
                   runtime_pack_digest=pack.manifest_digest, runtime_config_id=catalog.runtime_config.runtime_config_id)
    checkpoint = timing_mark(timing, checkpoint, 'render_batch_install')
    runtime = prepare_application(startup, pose_stage_factory=build_foundationpose_pose_stage).runtime
    checkpoint = timing_mark(timing, checkpoint, 'prepare_application')
    return dict(runtime=runtime, catalog=catalog, canonical_from_mesh=canonical_from_mesh,
                metadata={key: value for key, value in summary.items() if key != 'timing'}), checkpoint


def run(args, prepared=None):
    timing = dict(clock='perf_counter',
                  boundary='Python imports/CLI through close; excludes interpreter/SSH startup and summary write',
                  segments=[])
    checkpoint = timing_mark(timing, _SCRIPT_STARTED_S if prepared is None else time.perf_counter(),
                             'python_imports_and_cli')
    manifest = load_manifest(args.manifest)
    # Validate every capture before loading a model or creating output.
    inputs = []
    for row in manifest['frames']:
        path = args.manifest.parent / row['path']
        read_frame(path)
        inputs.append(dict(row, sha256=sha256(path)))
    checkpoint = timing_mark(timing, checkpoint, 'input_validation')
    summary = dict(schema_version='isaac_rgbd_detection.v1', status='INPUT_VALIDATED',
                   source=source_identity(), adapter_sha256=sha256(Path(__file__).resolve()),
                   manifest_sha256=sha256(args.manifest), inputs=inputs,
                   timing=timing,
                   measurement_boundary='offline RGB-D replay; camera extrinsics only; no object GT')
    checkpoint = timing_mark(timing, checkpoint, 'source_identity')
    if args.dry_run:
        print(json.dumps(summary, indent=2))
        return
    # Claim the output before entering cleanup; a raced existing directory is never written.
    args.run_dir.mkdir(parents=True, exist_ok=False)
    runtime = None
    valid_stamps = []
    window_observations = []
    try:
        if prepared is None:
            context, checkpoint = initialize_detection(args, summary, timing, checkpoint, manifest['frame_id'])
        else:
            context = prepared
            summary.update(context['metadata'])
            summary['model_reused'] = True
        runtime = context['runtime']
        if manifest['frame_id'] != context['catalog'].runtime_config.sensor_profile.optical_frame:
            raise ValueError('manifest optical frame differs from release')
        canonical_from_mesh = context['canonical_from_mesh']
        from online_detection_ros_node.sensor.depth_source import ImageSample, CameraInfoSample
        summary['tracking_session_id'] = runtime.tracking_session_id
        with (args.run_dir / 'observations.jsonl').open('x') as output:
            for row in inputs:
                path = args.manifest.parent / row['path']
                if sha256(path) != row['sha256']:
                    raise ValueError('capture changed since input validation')
                rgb, depth, k, position, q = read_frame(path)
                checkpoint = timing_mark(timing, checkpoint, 'frame_read_and_hash',
                                         capture_sequence_id=row['capture_sequence_id'])
                stamp, frame = row['source_stamp_ns'], manifest['frame_id']
                submission = runtime.submit_samples(ImageSample(stamp, stamp, frame, 'rgb8', rgb),
                    camera_info=CameraInfoSample(stamp, frame, rgb.shape[1], rgb.shape[0], tuple(k.flat), 'plumb_bob', (0.,) * 5),
                    depth=ImageSample(stamp, stamp, frame, '32FC1', depth))
                if not submission.accepted:
                    raise RuntimeError('runtime rejected sequential capture')
                result = runtime.take_result(timeout=args.result_timeout_s)
                if result is None or result.value is None or result.status.value == 'FAILED':
                    raise RuntimeError(f'perception failed or timed out: {result}')
                if not runtime.wait_until_idle(timeout=args.result_timeout_s):
                    raise TimeoutError('perception worker did not become idle')
                checkpoint = timing_mark(timing, checkpoint, 'frame_submit_and_wait',
                                         capture_sequence_id=row['capture_sequence_id'],
                                         worker_duration_s=runtime.stats.last_worker_duration_ns / 1e9)
                batch = result.value
                if batch.source_stamp_ns != stamp or batch.source_sequence_id != submission.frame.sequence_id or batch.frame_id != frame:
                    raise RuntimeError('result/capture identity mismatch')
                observation = extract(batch, position, q, canonical_from_mesh)
                observation['capture_sequence_id'] = row['capture_sequence_id']
                observation['diagnostics'] = dict(instances=[asdict(item) for item in batch.instances],
                    issues=batch.issues, admission=[asdict(item) for item in batch.admission_diagnostics])
                from PIL import Image
                Image.fromarray(render(rgb, batch, {'k': k.ravel().tolist()})).save(
                    args.run_dir / f"detection-{row['capture_sequence_id']:04d}.png")
                output.write(json.dumps(observation, allow_nan=False) + '\n')
                output.flush()
                window_observations.append(observation)
                if observation['pair_valid']:
                    valid_stamps.append(stamp)
                checkpoint = timing_mark(timing, checkpoint, 'frame_extract_render_output',
                                         capture_sequence_id=row['capture_sequence_id'])
        window = continuous_pair_window(window_observations)
        span = (window[-1] - window[0]) / 1e9 if len(window) >= 2 else 0
        summary.update(valid_pair_frames=len(valid_stamps),
                       valid_pair_span_s=(valid_stamps[-1] - valid_stamps[0]) / 1e9 if len(valid_stamps) >= 2 else 0,
                       continuous_valid_pair_span_s=span,
                       continuous_valid_pair_stamps_ns=window,
                       window_semantics="terminal consecutive captured samples with unchanged track/asset identity",
                       status='DETECTION_WINDOW_READY' if span >= 5 else 'INCOMPLETE_DETECTION_WINDOW',
                       runtime_stats=asdict(runtime.stats))
        if span < 5:
            raise RuntimeError('continuous valid coffee-machine/cup observations do not span 5 seconds')
    except Exception as error:
        summary.update(status='FAILED', error=str(error))
        raise
    finally:
        checkpoint = timing_mark(timing, checkpoint, 'finalize_or_error')
        if prepared is None:
            summary['cleanup_complete'] = runtime is None or runtime.close(timeout=12)
        else:
            summary['cleanup_complete'] = False
            try:
                prepared['runtime'] = runtime.new_session(timeout=12)
                summary['cleanup_complete'] = True
                summary['cleanup_boundary'] = 'request state reset; shared models retained by service'
            except Exception as error:
                summary.update(status='FAILED_CLEANUP', cleanup_error=str(error))
        timing_mark(timing, checkpoint, 'close' if prepared is None else 'reset_session')
        if not summary['cleanup_complete']:
            summary['status'] = 'FAILED_CLEANUP'
        if args.run_dir.is_dir():
            write_json(args.run_dir / 'summary.json', summary)
    if not summary['cleanup_complete']:
        raise RuntimeError('runtime cleanup incomplete')
    return summary


def self_test():
    import contextlib
    import io
    import tempfile
    from unittest.mock import patch
    from types import SimpleNamespace as NS
    from dataclasses import make_dataclass
    timing = {'segments': []}
    with patch.object(time, 'perf_counter', return_value=_SCRIPT_STARTED_S + 3), \
            patch.object(time, 'time', return_value=1234), contextlib.redirect_stdout(io.StringIO()) as captured:
        checkpoint = timing_mark(timing, _SCRIPT_STARTED_S + 1, 'frame_submit_and_wait',
                                 capture_sequence_id=30, worker_duration_s=1.5)
    assert checkpoint == _SCRIPT_STARTED_S + 3
    assert timing['segments'] == [dict(stage='frame_submit_and_wait', duration_s=2.0,
                                      elapsed_s=3.0, unix_s=1234, capture_sequence_id=30,
                                      worker_duration_s=1.5)]
    assert json.loads(captured.getvalue())['detection_timing'] == timing['segments'][0]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        frame = dict(rgb=np.zeros((2, 3, 3), np.uint8), depth_m=np.ones((2, 3), np.float32),
                     K=np.array([[2., 0, 1], [0, 2, 1], [0, 0, 1]]), position=np.array([1., 2., 3.]),
                     quaternion_wxyz=np.array([1., 0, 0, 0]))
        for index in range(2):
            np.savez(root / f'{index}.npz', **frame)
        manifest = dict(frame_id='camera', depth_convention='optical_z_m', camera_pose_convention='world_from_ros_optical',
                        frames=[dict(path=f'{i}.npz', source_stamp_ns=1 + i * 5_000_000_000, capture_sequence_id=i) for i in range(2)])
        path = root / 'manifest.json'
        write_json(path, manifest)
        assert len(load_manifest(path)['frames']) == 2
        read_frame(root / '0.npz')
        with patch('builtins.print') as printed:
            run(NS(manifest=path, dry_run=True))
        dry_summary = json.loads(printed.call_args.args[0])
        assert dry_summary['status'] == 'INPUT_VALIDATED'
        assert [row['stage'] for row in dry_summary['timing']['segments']] == [
            'python_imports_and_cli', 'input_validation', 'source_identity']
        assert all(row['duration_s'] >= 0 for row in dry_summary['timing']['segments'])
        assert world_pose([0, 0, 1], [1, 2, 3], [1, 0, 0, 0])['xyz_m'] == [1., 2., 4.]
        turned = world_pose([1, 0, 0], [0, 0, 0], [2 ** -.5, 0, 0, 2 ** -.5], [0, 0, 0, 1])
        assert np.allclose(turned['xyz_m'], [0, 1, 0])
        assert np.allclose(turned['quaternion_xyzw'], [0, 0, 2 ** -.5, 2 ** -.5])
        for key, value in [('path', '0.npz'), ('source_stamp_ns', 1), ('capture_sequence_id', 0)]:
            invalid = json.loads(json.dumps(manifest))
            invalid['frames'][1][key] = value
            write_json(path, invalid)
            try:
                load_manifest(path)
            except ValueError:
                pass
            else:
                raise AssertionError(f'accepted duplicate {key}')
        frame['depth_m'] = frame['depth_m'].astype(np.uint16)
        np.savez(root / 'bad.npz', **frame)
        try:
            read_frame(root / 'bad.npz')
        except ValueError:
            pass
        else:
            raise AssertionError('accepted non-metric depth encoding')
    # Contract fakes exercise rejection/output filtering only; never inference evidence.
    batch = NS(source_stamp_ns=1, source_sequence_id=2, frame_id='camera',
               tracking_session_id='test', runtime_config_id='test', provider_run_status=NS(value='OK'))
    asset = make_dataclass('Asset', [('asset_id', str)], frozen=True)('test')
    pose = NS(valid=True, provider_valid=True, freshness=NS(value='CURRENT'), state=NS(value='TRACKING'),
              age_ns=0, frame_id='camera', source_stamp_ns=1, source_sequence_id=2, position_xyz=(0, 0, 1),
              quaternion_xyzw=(0, 0, 0, 1), orientation_ambiguous=False, provider_id='foundationpose',
              provider_revision='test', model_revision='test', pose_asset=asset)
    machine = NS(track_id=1, lifecycle=NS(value='ACTIVE'), object_type='coffee_machine', pose=pose,
                 asset_match=NS(identity_valid=True))
    cup = NS(track_id=2, lifecycle=NS(value='ACTIVE'), object_type='cup', pose=pose,
             asset_match=NS(identity_valid=True), center_3d=pose)
    batch.instances = [machine, cup]
    transforms = {asset: np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]).ravel()}
    result = extract(batch, [0, 0, 0], [1, 0, 0, 0], transforms)
    assert np.allclose(result['tracks'][0]['world']['quaternion_xyzw'], [0, 0, 2**-.5, 2**-.5])
    assert result['pair_valid'] and 'quaternion_xyzw' not in result['tracks'][1]
    cup.asset_match.identity_valid = False
    assert not extract(batch, [0, 0, 0], [1, 0, 0, 0], transforms)['pair_valid']
    cup.asset_match.identity_valid = True
    pose.orientation_ambiguous = True
    assert not extract(batch, [0, 0, 0], [1, 0, 0, 0], transforms)['pair_valid']
    pose.orientation_ambiguous = False
    cup.pose = NS(**vars(pose))
    cup.pose.orientation_ambiguous = True
    assert extract(batch, [0, 0, 0], [1, 0, 0, 0], transforms)['pair_valid']
    cup.pose = pose
    pose.source_stamp_ns = 0
    assert not extract(batch, [0, 0, 0], [1, 0, 0, 0], transforms)['tracks']
    observations = [dict(result, source_stamp_ns=1 + i * 1_000_000_000) for i in range(6)]
    assert len(continuous_pair_window(observations)) == 6
    observations[2] = dict(observations[2], pair_valid=False)
    assert len(continuous_pair_window(observations)) == 3
    observations[4] = dict(observations[4], tracking_session_id='restarted')
    assert len(continuous_pair_window(observations)) == 1
    print('Isaac RGB-D adapter CPU boundary check: PASS (no inference)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serve', action='store_true')
    parser.add_argument('--request', action='store_true')
    parser.add_argument('--service-root', type=Path)
    parser.add_argument('--socket', type=Path)
    parser.add_argument('--owner-token')
    parser.add_argument('--expected-identity')
    parser.add_argument('--allowed-read-root', type=Path, action='append')
    parser.add_argument('--allowed-output-root', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--node-config', type=Path)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--dry-run', action='store_true', help='validate captures only; no model load or GPU')
    parser.add_argument('--self-test', action='store_true', help='CPU boundary checks; no model load or GPU')
    parser.add_argument('--render-batch-size', type=int, default=8)
    parser.add_argument('--result-timeout-s', type=float, default=120)
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.serve or args.request:
        if args.serve and args.request:
            parser.error('--serve and --request are mutually exclusive')
        from isaac_detection_service import serve, request
        return serve(args, sys.modules[__name__]) if args.serve else request(args)
    if args.manifest is None or (not args.dry_run and (args.node_config is None or args.run_dir is None)):
        parser.error('--manifest required; inference also requires --node-config and --run-dir')
    if args.render_batch_size <= 0 or not math.isfinite(args.result_timeout_s) or args.result_timeout_s <= 0:
        parser.error('batch size and timeout must be positive')
    for key in ('manifest', 'node_config', 'run_dir'):
        if getattr(args, key) is not None:
            setattr(args, key, getattr(args, key).resolve())
    run(args)


if __name__ == '__main__':
    main()
