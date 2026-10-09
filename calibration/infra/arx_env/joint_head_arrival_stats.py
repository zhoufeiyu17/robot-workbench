"""Persistent arrival errors, independent of calibration image-batch retention."""
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import statistics
import tempfile


class ArrivalStats:
    def __init__(self, root, robot_id, robot_source):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError('robot_id must be a nonempty string')
        if robot_source not in ('physical', 'simulated'):
            raise ValueError('robot_source must be physical or simulated')
        self.root = Path(root).resolve()
        self.robot_id, self.robot_source = robot_id, robot_source
        self.root.mkdir(parents=True, exist_ok=True)
        self.samples_path = self.root / 'samples.jsonl'
        with self._locked():
            self._check_identity()
            self._write_summary(self._load())

    @contextmanager
    def _locked(self):
        with (self.root / '.lock').open('a+b') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream, fcntl.LOCK_UN)

    def _check_identity(self):
        path = self.root / 'summary.json'
        if path.exists():
            self._identity(json.loads(path.read_text()))

    def _identity(self, row):
        if not isinstance(row, dict):
            raise ValueError('arrival sample must be an object')
        if row.get('robot_id') != self.robot_id or row.get('robot_source') != self.robot_source:
            raise ValueError('arrival statistics cannot mix robot identities or sources')

    def _validate(self, sample):
        if not isinstance(sample, dict):
            raise ValueError('sample must be an object')
        row = dict(sample)
        for key in ('event_id', 'batch_id', 'workflow', 'plan_sha256'):
            if not isinstance(row.get(key), str) or not row[key]:
                raise ValueError(key + ' must be a nonempty string')
        if row.get('status') not in ('success', 'failed'):
            raise ValueError('status must be success or failed')
        for key in ('timestamp_ns', 'owner_pid', 'command_epoch'):
            value = row.get(key)
            if type(value) is not int or value < (0 if key == 'command_epoch' else 1):
                raise ValueError(key + ' must be a valid integer')
        if 'reason' in row and row['reason'] is not None and not isinstance(row['reason'], str):
            raise ValueError('reason must be text or null')
        for key in ('q_target_rad', 'q_actual_rad'):
            values = row.get(key)
            if (not isinstance(values, (list, tuple)) or len(values) != 12 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError(key + ' must contain 12 finite numbers')
            row[key] = [float(v) for v in values]
        row['error_rad'] = [a - t for a, t in zip(row['q_actual_rad'], row['q_target_rad'])]
        if not all(math.isfinite(v) for v in row['error_rad']):
            raise ValueError('arrival errors must be finite')
        for key, value in (('robot_id', self.robot_id), ('robot_source', self.robot_source)):
            if key in row and row[key] != value:
                raise ValueError('arrival statistics cannot mix robot identities or sources')
            row[key] = value
        # Freeze caller-owned values and reject any non-JSON/nonfinite metadata.
        return json.loads(json.dumps(row, allow_nan=False))

    def _load(self):
        rows = {}
        if not self.samples_path.exists():
            return rows
        raw = self.samples_path.read_bytes()
        offset = 0
        for line in raw.splitlines(keepends=True):
            try:
                row = json.loads(line)
            except (ValueError, UnicodeDecodeError) as exc:
                if offset + len(line) == len(raw) and not line.endswith(b'\n'):
                    # An interrupted append is not a committed sample; repair before appending.
                    with self.samples_path.open('r+b') as stream:
                        stream.truncate(offset)
                        stream.flush()
                        os.fsync(stream.fileno())
                    break
                raise ValueError('corrupt arrival sample at byte ' + str(offset)) from exc
            self._identity(row)
            checked = self._validate(row)
            if checked != row:
                raise ValueError('corrupt arrival sample: inconsistent error_rad')
            previous = rows.get(row['event_id'])
            if previous is not None and previous != row:
                raise ValueError('conflicting arrival event_id: ' + row['event_id'])
            rows[row['event_id']] = row
            offset += len(line)
        return rows

    @staticmethod
    def _joint(values):
        if not values:
            return dict(count=0, mean_rad=None, std_rad=None, rms_rad=None, abs_p95_rad=None, abs_max_rad=None)
        absolute = sorted(abs(v) for v in values)
        position = (len(values) - 1) * .95
        low, high = math.floor(position), math.ceil(position)
        return dict(count=len(values), mean_rad=math.fsum(v / len(values) for v in values), std_rad=statistics.pstdev(values),
                    rms_rad=(math.sqrt(statistics.fmean((v / absolute[-1]) ** 2 for v in values)) * absolute[-1] if absolute[-1] else 0.),
                    abs_p95_rad=absolute[low] + (absolute[high] - absolute[low]) * (position - low),
                    abs_max_rad=absolute[-1])

    def _summarize(self, rows):
        # Legacy failure records remain audit evidence, never arrival samples.
        selected = [row['error_rad'] for row in rows.values() if row['status'] == 'success']
        joints = {side: [self._joint([row[index] for row in selected]) for index in range(start, start + 6)]
                  for side, start in (('left', 0), ('right', 6))}
        return dict(schema='joint-head-arrival-stats-v2', robot_id=self.robot_id,
                    robot_source=self.robot_source, units='rad', error_definition='actual-target',
                    total_count=len(selected), joints=joints, path=str(self.root),
                    raw_path=str(self.samples_path), summary_path=str(self.root / 'summary.json'))

    def _write_summary(self, rows):
        result = self._summarize(rows)
        fd, name = tempfile.mkstemp(prefix='.summary-', suffix='.tmp', dir=self.root)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(result, stream, allow_nan=False, separators=(',', ':'))
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self.root / 'summary.json')
            directory = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(name):
                os.unlink(name)
        return result

    def record(self, sample):
        row = self._validate(sample)
        if row['status'] != 'success':
            return False
        with self._locked():
            self._check_identity()
            # ponytail: reread/sort the complete log per write; use indexed aggregates if history makes this slow.
            rows = self._load()
            if row['event_id'] in rows:
                if rows[row['event_id']] != row:
                    raise ValueError('conflicting arrival event_id: ' + row['event_id'])
                self._write_summary(rows)
                return False
            payload = (json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n').encode()
            with self.samples_path.open('a+b') as stream:
                stream.seek(0, os.SEEK_END)
                if stream.tell():
                    stream.seek(-1, os.SEEK_END)
                    if stream.read(1) != b'\n':
                        stream.write(b'\n')
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            rows[row['event_id']] = row
            self._write_summary(rows)
            return True

    def summary(self):
        with self._locked():
            self._check_identity()
            return self._write_summary(self._load())
