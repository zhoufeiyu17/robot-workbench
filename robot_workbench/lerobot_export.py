"""CPU-only LeRobot export from sealed physical raw; no SDK or detector imports."""
from bisect import bisect_left, bisect_right
import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys

CAMERAS = ('base_0', 'left_wrist_0', 'right_wrist_0')
AXES = [f'{side}_J{i}' for side in ('left', 'right') for i in range(1, 7)]
AXES = AXES[:6] + ['left_jaw_mm'] + AXES[6:] + ['right_jaw_mm']


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def verified_export(output, raw_roots, *, repo_id=None, task=None, fps=None):
    """Reuse only pinned completed exports; an interrupted output is never overwritten."""
    output = Path(output)
    receipt_path = output / 'audit/export-receipt.json'
    if receipt_path.is_symlink() or not receipt_path.is_file() or receipt_path.stat().st_size > 4 * 1024**2:
        raise ValueError('existing dataset has no bounded completed export receipt; preserve it and use a new output')
    receipt = json.loads(receipt_path.read_text())
    pins = [digest(Path(root) / 'manifest.json') for root in raw_roots]
    if (receipt.get('schema') != 'workbench.lerobot_export.v1' or receipt.get('status') != 'NATIVE_READBACK_PASS'
            or [e['raw_manifest_sha256'] for e in receipt.get('episodes', [])] != pins
            or any(value is not None and receipt.get(key)!=value for key,value in (('repo_id',repo_id),('task',task),('fps',fps)))
            or not receipt.get('files')):
        raise ValueError('existing dataset differs from this raw recording')
    for name, pin in receipt['files'].items():
        relative = Path(name)
        path = output / relative
        if (relative.is_absolute() or '..' in relative.parts or path.is_symlink() or not path.is_file()
                or not path.resolve().is_relative_to(output.resolve())
                or path.stat().st_size != pin['bytes'] or digest(path) != pin['sha256']):
            raise ValueError('saved LeRobot file changed: ' + name)
    return receipt


def vector(value, disabled=()):
    if (value.get('schema_version') != 'physical14.v1' or value.get('encoding') != 'absolute'
            or value.get('units') != dict(joints='rad', grippers='mm')):
        raise ValueError('physical14 rad/mm values required')
    result, valid = [], []
    for side in ('left', 'right'):
        joints = value.get(side + '_arm_joints')
        if not isinstance(joints, list) or len(joints) != 6:
            raise ValueError(side + ' requires six joint values')
        result.extend(joints)
        valid.extend([True] * 6)
        jaw = value.get(side + '_gripper')
        if side in disabled:
            result.append(0.)
            valid.append(False)
        else:
            result.append(jaw)
            valid.append(True)
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in result):
        raise ValueError('nonfinite or missing enabled-axis value')
    return result, valid


def _times(rows, field):
    times = [int(row[field]) for row in rows]
    if not times or times[0] <= 0 or any(a >= b for a, b in zip(times, times[1:])):
        raise ValueError('strictly advancing original timestamps required: ' + field)
    return times


def alignment(commands, feedback, cameras, *, fps=30, disabled=(), max_gap_ns=100_000_000,
              max_camera_offset_ns=50_000_000):
    """Uniform training grid, bounded feedback interpolation, nearest actual RGB frames."""
    if type(fps) is not int or not 1 <= fps <= 100:
        raise ValueError('integer sampling rate from 1 to 100 Hz required')
    ct = _times(commands, 'send_wall_ns')
    ft = _times(feedback, 'sample_wall_ns')
    frames = {key: _times(cameras[key], 'host_receive_ns') for key in CAMERAS}
    if len(ft) < 2 or ft[0] > ct[0] or ft[-1] < ct[-1]:
        raise ValueError('feedback must cover the complete command interval')
    end_index = (ct[-1] - ct[0]) * fps // 1_000_000_000
    for index in range(end_index + 1):
        stamp = ct[0] + round(index * 1_000_000_000 / fps)
        command = bisect_right(ct, stamp) - 1
        right = min(bisect_left(ft, stamp), len(ft) - 1)
        left = right if ft[right] == stamp else right - 1
        if left < 0 or ft[right] - ft[left] > max_gap_ns:
            raise ValueError('feedback gap exceeds the export alignment bound')
        a, am = vector(feedback[left]['state'], disabled)
        b, bm = vector(feedback[right]['state'], disabled)
        weight = 0. if right == left else (stamp - ft[left]) / (ft[right] - ft[left])
        state = [x + (y - x) * weight for x, y in zip(a, b)]
        target, target_mask = vector(commands[command]['target'], disabled)
        selected = {}
        for camera, times in frames.items():
            pos = bisect_left(times, stamp)
            choices = [i for i in (pos - 1, pos) if 0 <= i < len(times)]
            chosen = min(choices, key=lambda i: (abs(times[i] - stamp), i))
            offset = times[chosen] - stamp
            if abs(offset) > max_camera_offset_ns:
                raise ValueError(camera + ': no captured frame within the export alignment bound')
            selected[camera] = dict(index=chosen, offset_ns=offset,
                host_receive_ns=times[chosen], frame_number=cameras[camera][chosen]['frame_number'])
        yield dict(index=index, timestamp_s=index / fps, sample_wall_ns=stamp,
            command_index=command, command_wall_ns=ct[command], feedback_indices=[left, right],
            feedback_wall_ns=[ft[left], ft[right]], state=state, state_valid=[x and y for x, y in zip(am, bm)],
            action=target, action_valid=target_mask, cameras=selected)


def _rows(path):
    with path.open() as stream:
        for line in stream:
            if len(line) > 256 * 1024:
                raise ValueError('bounded raw event required')
            yield json.loads(line)


class FrameReader:
    def __init__(self, iterator):
        self.iterator, self.index, self.value = iterator, -1, None

    def get(self, index):
        if index < self.index:
            raise ValueError('frame selection cannot move backwards')
        while self.index < index:
            self.value = next(self.iterator)
            self.index += 1
        return self.value


def export(raw_roots, output, *, repo_id, task, fps=30):
    """Unique output only; failures retain raw and partial exports for diagnosis."""
    from real_recording import verify_raw, iter_rgbd
    import numpy as np
    from PIL import Image
    os.environ.update(CUDA_VISIBLE_DEVICES='', HF_HUB_OFFLINE='1', HF_DATASETS_OFFLINE='1')
    output = Path(output)
    if not output.is_absolute() or output.is_symlink():
        raise ValueError('absolute regular dataset root required')
    if not raw_roots or len(set(map(str, raw_roots))) != len(raw_roots) or not task.strip():
        raise ValueError('distinct raw episodes and an explicit task description required')
    if output.exists():
        return verified_export(output, raw_roots, repo_id=repo_id, task=task, fps=fps)
    episodes, asset_selections = [], {}
    for path in map(Path, raw_roots):
        if output.resolve().is_relative_to(path.resolve()):
            raise ValueError('dataset output must remain outside sealed raw')
        manifest, source, _ = verify_raw(path, environment='physical')
        if source.get('schema_version')!='physical_collection_source.v1' or source.get('training_observation') is False:
            raise ValueError('only collection training raw may enter this dataset; maintenance return is excluded')
        selection = source.get('scene', {}).get('asset_set')
        if selection is not None:
            from scene_assets import validate
            asset_selections[path] = validate(selection)
        if (manifest.get('capture_status') != 'complete' or manifest.get('controller_outcome') != 'completed'
                or manifest.get('validation', {}).get('status') != 'passed'):
            raise ValueError('only completed, data-valid raw can become a successful dataset episode')
        commands, feedback = list(_rows(path / 'commands.jsonl')), list(_rows(path / 'feedback.jsonl'))
        cameras = {key: list(_rows(path / (key + '-pairs.jsonl'))) for key in CAMERAS}
        rows = list(alignment(commands, feedback, cameras, fps=fps,
            disabled=source.get('initial_state', {}).get('disabled_grippers', [])))
        episodes.append((path, manifest, rows))
    from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION
    if CODEBASE_VERSION != 'v2.1':
        raise ValueError('this exporter is qualified only for the installed LeRobot v2.1 format')
    first_root, _, first_rows = episodes[0]
    _, _, first_png, _ = FrameReader(iter_rgbd(first_root, CAMERAS[0])).get(first_rows[0]['cameras'][CAMERAS[0]]['index'])
    with Image.open(io.BytesIO(first_png)) as image:
        width, height = image.size
    features = {key: dict(dtype=dtype, shape=(14,), names=AXES) for key, dtype in (
        ('observation.state', 'float32'), ('action', 'float32'), ('observation.state_valid', 'bool'), ('action_valid', 'bool'))}
    features.update({f'observation.images.{key}': dict(dtype='video', shape=(height, width, 3),
        names=['height', 'width', 'channels']) for key in CAMERAS})
    dataset = LeRobotDataset.create(repo_id=repo_id, root=output, fps=fps, robot_type='arx6',
        features=features, use_videos=True, image_writer_processes=0, image_writer_threads=4, video_backend='pyav')
    # Bound the native v2.1 writer's stdlib Queue before submitting any pixels.
    dataset.image_writer.queue.maxsize = 32
    audit = output / 'audit'
    audit.mkdir()
    expected, receipts = [], []
    for episode_index, (root, manifest, rows) in enumerate(episodes):
        readers = {key: FrameReader(iter_rgbd(root, key)) for key in CAMERAS}
        with (audit / f'episode_{episode_index:06d}.jsonl').open('x') as stream:
            for row in rows:
                frame = {key: np.asarray(row[name], dtype=dtype) for key, name, dtype in (
                    ('observation.state', 'state', 'float32'), ('action', 'action', 'float32'),
                    ('observation.state_valid', 'state_valid', 'bool'), ('action_valid', 'action_valid', 'bool'))}
                for key, reader in readers.items():
                    _, meta, png, _ = reader.get(row['cameras'][key]['index'])
                    if int(meta['host_receive_ns']) != row['cameras'][key]['host_receive_ns']:
                        raise ValueError('decoded camera sample differs from the alignment index')
                    with Image.open(io.BytesIO(png)) as image:
                        if image.size != (width, height):
                            raise ValueError('camera shape changed inside the dataset')
                        frame['observation.images.' + key] = np.asarray(image.convert('RGB'))
                dataset.add_frame(frame, task=task, timestamp=row['timestamp_s'])
                stream.write(json.dumps(row, allow_nan=False) + '\n')
                expected.append(dict(action=frame['action'], state=frame['observation.state']))
        dataset.save_episode()
        receipts.append(dict(episode_index=episode_index, frames=len(rows), raw_root=str(root),
            raw_manifest_sha256=digest(root / 'manifest.json'),
            source_run_id=manifest['source_run_id'], execution_timeline_digest=manifest.get('execution_timeline_digest')))
        if root in asset_selections:
            receipts[-1]['scene_asset_set'] = asset_selections[root]
    reader = LeRobotDataset(repo_id=repo_id, root=output, download_videos=False, video_backend='pyav')
    if len(reader) != len(expected) or reader.num_episodes != len(episodes):
        raise ValueError('native LeRobot readback episode/frame counts differ')
    table = reader.hf_dataset.select_columns(['action', 'observation.state'])
    for index, row in enumerate(table):
        for key, name in (('action', 'action'), ('observation.state', 'state')):
            value = row[key].numpy() if hasattr(row[key], 'numpy') else row[key]
            if not np.array_equal(value, expected[index][name]):
                raise ValueError('native LeRobot numeric readback differs at frame ' + str(index))
    for index in {0, len(reader) // 2, len(reader) - 1}:
        row = reader[index]
        for key in CAMERAS:
            if tuple(row['observation.images.' + key].shape) != (3, height, width):
                raise ValueError('native LeRobot video readback shape differs')
    import av
    videos = []
    for episode in receipts:
        for camera in CAMERAS:
            path = output / reader.meta.get_video_file_path(episode['episode_index'], 'observation.images.' + camera)
            count = 0
            with av.open(str(path)) as container:
                for count, frame in enumerate(container.decode(video=0), 1):
                    if ((frame.width, frame.height) != (width, height) or frame.pts is None
                            or abs(float(frame.pts * frame.time_base) - (count - 1) / fps) > 1e-5):
                        raise ValueError('encoded LeRobot video geometry or timing differs')
            if count != episode['frames']:
                raise ValueError('encoded LeRobot video lost or duplicated sample frames')
            videos.append(dict(path=str(path.relative_to(output)), frames=count,
                sha256=digest(path)))
    receipt = dict(schema='workbench.lerobot_export.v1', status='NATIVE_READBACK_PASS',
        format_version=CODEBASE_VERSION, repo_id=repo_id, task=task, root=str(output), fps=fps, episodes=receipts,
        total_frames=len(reader), axes=AXES, units=dict(joints='rad', grippers='mm'),
        missing_axis_policy='finite zero sentinel only where the corresponding validity mask is false',
        action_semantics='recorded absolute target reference; not a measured position or firmware acknowledgement',
        time_policy='uniform sample grid; original command/feedback/camera clocks retained in audit JSONL',
        rgbd_policy='RGB videos in LeRobot; original aligned depth stays in sealed raw',
        videos=videos,
        model_loads=0, robot_commands=0, hub_upload=False)
    receipt['files'] = {str(path.relative_to(output)): dict(bytes=path.stat().st_size, sha256=digest(path))
        for directory in ('data', 'meta', 'videos', 'audit') for path in sorted((output / directory).rglob('*')) if path.is_file()}
    (audit / 'export-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rule_based_workbench'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', action='append', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--task', required=True)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()
    print(json.dumps(export(args.raw_root, args.output, repo_id=args.repo_id, task=args.task, fps=args.fps)))
