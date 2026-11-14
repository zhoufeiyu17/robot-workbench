"""Bounded numeric probes and an independent, read-only incident collector."""
from collections import deque
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
import urllib.request

SCHEMA = 'workbench.runtime-monitor.v1'
SAMPLE_NS = 50_000_000
WINDOW_NS = 6_000_000_000
MAX_CHANNELS = 96
MAX_SAMPLES = 128
MAX_INCIDENT_BYTES = 4 * 1024 * 1024
MAX_INCIDENTS = 16


class RuntimeProbes:
    def __init__(self, *, clock=time.monotonic_ns):
        self.clock = clock
        self.lock = threading.Lock()
        self.channels = {}
        self.faults = deque(maxlen=32)
        self.sequence = self.fault_sequence = self.dropped = 0
        self.last_fault = {}
        try:
            self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:
            self.boot_id = None

    def mark(self, channel, *, force=False, now_ns=None, **fields):
        # A busy reader can lose telemetry; it cannot make a producer wait.
        if not self.lock.acquire(blocking=False):
            self.dropped += 1
            return
        try:
            if (not isinstance(channel, str) or len(channel) > 192 or len(fields) > 20
                    or any(type(v) not in (str, int, float, bool, type(None))
                           or isinstance(v, str) and len(v) > 256
                           or type(v) is float and not math.isfinite(v) for v in fields.values())):
                self.dropped += 1
                return
            now = self.clock() if now_ns is None else now_ns
            row = self.channels.get(channel)
            if row is None:
                if len(self.channels) >= MAX_CHANNELS:
                    retired = next((key for key, old in self.channels.items()
                        if not old['latest'].get('active') and now-old['latest']['at_ns']>WINDOW_NS), None)
                    if retired is None:
                        self.dropped += 1
                        return
                    del self.channels[retired]
                row = self.channels[channel] = dict(latest=None, history=deque(maxlen=MAX_SAMPLES),
                    edges=deque(maxlen=MAX_SAMPLES), edge_drops=0, sampled_ns=0)
            self.sequence += 1
            point = dict(at_ns=now, sequence=self.sequence, **fields)
            row['latest'] = point
            if now - row['sampled_ns'] >= SAMPLE_NS:
                row['history'].append(point)
                row['sampled_ns'] = now
            if force:
                if len(row['edges'])==MAX_SAMPLES and now-row['edges'][0]['at_ns']<WINDOW_NS:
                    row['edge_drops']+=1
                row['edges'].append(point)
        finally:
            self.lock.release()

    def fault(self, reason, **identity):
        if not self.lock.acquire(blocking=False):
            self.dropped += 1
            return
        try:
            if len(identity) > 16 or any(type(v) not in (str, int, float, bool, type(None)) for v in identity.values()):
                self.dropped += 1
                return
            now = self.clock()
            key = (reason, identity.get('run_id'), identity.get('epoch'))
            if now - self.last_fault.get(key, -WINDOW_NS) < WINDOW_NS:
                return
            if len(self.last_fault) >= 128:
                self.last_fault.clear()
            self.last_fault[key] = now
            self.fault_sequence += 1
            self.faults.append(dict(id=self.fault_sequence, at_ns=now, reason=str(reason)[:256], **identity))
        finally:
            self.lock.release()

    @contextmanager
    def span(self, channel, stage, **identity):
        self.mark(channel, force=True, active=True, stage=stage, **identity)
        try:
            yield
        except BaseException:
            self.mark(channel, force=True, active=False, stage='failed', **identity)
            raise
        else:
            self.mark(channel, force=True, active=False, stage='complete', **identity)

    def snapshot(self, *, history=False):
        now = self.clock()
        # Copy only references while holding the producer lock. JSON stays in the reader.
        with self.lock:
            rows = [(key, row['latest'], tuple(row['history'])+tuple(row['edges']) if history else (), row['edge_drops'])
                    for key, row in self.channels.items()]
            faults = list(self.faults)
            sequence, dropped = self.sequence, self.dropped
        channels = {}
        for key, latest, points, edge_drops in rows:
            channels[key] = dict(latest=latest, edge_history_dropped=edge_drops)
            if history:
                retained={p['sequence']:p for p in points if p['at_ns'] >= now - WINDOW_NS}
                channels[key]['history'] = [retained[k] for k in sorted(retained)]
        return dict(schema=SCHEMA, pid=os.getpid(), boot_id=self.boot_id, sampled_ns=now,
                    sampled_wall_ns=time.time_ns(), sequence=sequence, dropped=dropped,
                    sample_interval_ns=SAMPLE_NS, history_window_ns=WINDOW_NS,
                    channels=channels, faults=faults)


probes = RuntimeProbes()


def analyze(snapshot):
    """Report observed stalls; coincident MEM activity is correlation evidence."""
    now = snapshot['sampled_ns']
    findings = []
    latest = {key: row['latest'] for key, row in snapshot['channels'].items()}
    for key, row in latest.items():
        if row.get('kind')=='raw' and row.get('receive_age_ns') is not None and row['receive_age_ns']>100_000_000:
            findings.append(dict(kind='source_delivery_lag', channel=key, age_ms=row['receive_age_ns']/1e6,
                                 source_sequence=row.get('source_sequence')))
        if row.get('active') and now - row['at_ns'] >= 100_000_000:
            findings.append(dict(kind='active_stage', channel=key, age_ms=(now-row['at_ns'])/1e6,
                                 stage=row.get('stage'), request_id=row.get('request_id')))
        if row.get('kind') == 'selection':
            for role in ('head', 'left', 'right', 'state'):
                used = row.get(role + '_selected_ns')
                available = row.get(role + '_latest_ns')
                if used is not None and available is not None and available - used > 100_000_000:
                    findings.append(dict(kind='selection_behind_available', channel=key, stream=role,
                                         lag_ms=(available-used)/1e6))
        if row.get('kind') == 'ready':
            prefix = key.rsplit('/', 1)[0]
            raw = latest.get(prefix + '/raw', {})
            metadata = latest.get(prefix + '/metadata', {})
            if raw.get('source_ns', 0) - row.get('source_ns', 0) > 100_000_000:
                stage = ('metadata_pending' if metadata.get('source_ns', 0) < raw['source_ns'] else 'metadata_join')
                if not raw.get('requires_metadata', bool(metadata)):
                    stage = 'cache_admission'
                findings.append(dict(kind='image_ready_lag', channel=key,
                                     stage=stage,
                                     raw_source_ns=raw.get('source_ns'), metadata_source_ns=metadata.get('source_ns'),
                                     ready_source_ns=row.get('source_ns')))
    commits = snapshot['channels'].get('memory/write', {}).get('history', [])
    previous = {}
    for row in commits:
        if row.get('stage')!='committed' or not row.get('stride'):
            continue
        key = (row.get('generation'), row.get('kind'))
        old = previous.get(key)
        if old and (row['step']-old['step']) % row['stride']:
            findings.append(dict(kind='memory_stride_mismatch', channel='memory/write',
                                 previous_step=old['step'], step=row['step'], stride=row['stride']))
        previous[key] = row
    return findings


def memory_issues(previous, current):
    """Shared checks previously performed by the temporary MEM observer."""
    if not current:
        return []
    issues = []
    if current.get('prediction_commits_memory') or current.get('predictions_used_as_actions', 0):
        issues.append('prediction unexpectedly commits memory')
    if previous and previous.get('revision') == current.get('revision'):
        if current.get('observations', 0) < previous.get('observations', 0):
            issues.append('memory observation count regressed within revision')
        if current.get('source_gaps', 0) > previous.get('source_gaps', 0):
            issues.append('memory source gap increased')
    if current.get('error'):
        issues.append(str(current['error'])[:256])
    return issues


def replay_incident(incident):
    """Re-evaluate saved numeric history without cameras, model, or SDK access."""
    result = []
    views = {}
    for window in ('before', 'after'):
        for source, view in (incident.get(window) or {}).items():
            key = (source, view['boot_id'], view['pid'])
            views.setdefault(key, {})
            for channel, row in view['channels'].items():
                for point in [*row.get('history', []), row['latest']]:
                    views[key][(point['sequence'], channel)] = point
    for (source, boot, pid), rows in views.items():
        channels, active = {}, set()
        for (_, channel), point in sorted(rows.items(), key=lambda pair: pair[1]['at_ns']):
            history = channels.get(channel, {}).get('history', [])[-127:] + [point]
            channels[channel] = dict(latest=point, history=history)
            findings = analyze(dict(sampled_ns=point['at_ns'], channels=channels))
            current = {(f['kind'], f['channel'], f.get('stage')) for f in findings}
            for finding in findings:
                key = (finding['kind'], finding['channel'], finding.get('stage'))
                if key not in active:
                    result.append(dict(source=source, boot_id=boot, pid=pid, at_ns=point['at_ns'], **finding))
            active = current
    return result


class IncidentCollector:
    """Runs outside the business process. It can only GET and write its own directory."""
    def __init__(self, endpoint, output, *, clock=time.monotonic):
        self.endpoint = endpoint.rstrip('/')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.pending = {}
        self.seen = set()
        self.memory = None
        self.memory_identity = None
        self.read_errors = self.write_errors = self.saved = self.omitted = 0
        self.last_error = None
        self.last_write_error = None
        self.last_status = None
        self.status_written_at = 0.

    def get(self, history=False):
        query = '/api/inference-diagnostics?environment=physical&limit=0&monitor_only=1&monitor_history=' + str(int(history))
        with self.opener.open(self.endpoint + query, timeout=2) as response:
            raw = response.read(8 * 1024 * 1024 + 1)
        if len(raw) > 8 * 1024 * 1024:
            raise ValueError('monitor response exceeds 8 MiB')
        return json.loads(raw)

    @staticmethod
    def sources(reply):
        values = {'business': reply['business']['monitor']}
        owner = reply.get('owner', {})
        if owner.get('status') == 'observed' and 'monitor' in owner.get('data', {}):
            values['owner'] = owner['data']['monitor']
        return values

    def _write(self, name, value):
        encoded = json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode()
        if len(encoded) > MAX_INCIDENT_BYTES:
            self.omitted += 1
            raise ValueError('incident exceeds bounded file size')
        temporary = self.output / (name + '.tmp')
        temporary.write_bytes(encoded)
        temporary.replace(self.output / name)

    def poll(self):
        try:
            reply = self.get()
            sources = self.sources(reply)
            context = reply.get('context', {})
            identity = (context.get('run_id'), context.get('model_id'))
            memory = context.get('memory')
            issues = memory_issues(self.memory if identity == self.memory_identity else None, memory)
            self.memory, self.memory_identity = memory, identity
            new = []
            for source, view in sources.items():
                for fault in view['faults']:
                    key = (source, view['boot_id'], view['pid'], fault['id'])
                    if key not in self.seen:
                        self.seen.add(key)
                        new.append((key, fault))
            if len(self.seen) > 4096:
                self.seen = {(source, view['boot_id'], view['pid'], fault['id'])
                             for source, view in sources.items() for fault in view['faults']}
            if new:
                detailed = self.get(history=True)
                for key, fault in new:
                    self.pending[key] = dict(schema='workbench.incident.v1', fault=fault, source=key[0],
                        context=detailed.get('context'), before=self.sources(detailed), after=None,
                        detected_wall=datetime.now(timezone.utc).isoformat(), memory_issues=issues)
                self.flush(ready_only=False)
            if any(self.clock() >= incident['finish_at'] for incident in self.pending.values()):
                detailed = self.sources(self.get(history=True))
                for incident in self.pending.values():
                    if self.clock() >= incident['finish_at']:
                        incident['after'] = detailed
                self.flush(ready_only=True)
            self.last_error = None
            status = dict(schema='workbench.monitor-status.v1', healthy=self.last_write_error is None, read_errors=self.read_errors,
                          write_errors=self.write_errors, saved=self.saved, omitted=self.omitted,
                          write_error=self.last_write_error,
                          pending=len(self.pending), memory_issues=issues,
                          missing_owner_monitor='owner' not in sources,
                          sources={k: dict(pid=v['pid'], boot_id=v['boot_id'], dropped=v['dropped'],
                                           findings=analyze(v)) for k, v in sources.items()})
            # Normal polls update one small atomic status, never a full timeline.
            if status != self.last_status or self.clock()-self.status_written_at >= 30.:
                self._write('status.json', dict(status, checked_wall_ns=time.time_ns()))
                self.last_status = status
                self.status_written_at = self.clock()
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.read_errors += 1
            self.last_error = str(error)[:256]
            try:
                self._write('status.json', dict(healthy=False, read_errors=self.read_errors,
                            write_errors=self.write_errors, error=self.last_error))
            except (OSError, ValueError):
                self.write_errors += 1

    def flush(self, *, ready_only):
        for key, incident in list(self.pending.items()):
            if 'finish_at' not in incident:
                incident['finish_at'] = self.clock() + 1.
                incident['slot'] = self.saved % MAX_INCIDENTS
                self.saved += 1
            if ready_only and incident['after'] is None:
                continue
            name = 'incident-%02d.json' % incident['slot']
            value = {k: v for k, v in incident.items() if k not in ('finish_at', 'slot')}
            value['complete_post_window'] = incident['after'] is not None
            try:
                self._write(name, value)
            except (OSError, ValueError) as error:
                self.write_errors += 1
                self.last_write_error = str(error)[:256]
            else:
                self.last_write_error = None
            if ready_only:
                del self.pending[key]
