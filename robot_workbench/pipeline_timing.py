"""Bounded, payload-free pipeline measurements; never used for control decisions."""
from collections import deque
import math
import os
from pathlib import Path
import secrets
import threading
import time
from .runtime_monitor import probes

SCHEMA = 'workbench.pipeline-timing.v1'


class PipelineTiming:
    def __init__(self, capacity=8192, sample_count=256):
        self.capacity, self.sample_count = capacity, sample_count
        self.events = deque(maxlen=capacity)
        self.stages = {}
        self.faults = deque(maxlen=4)
        self.lock = threading.Lock()
        self.sequence = self.invalid = self.dropped = 0
        self.instance = secrets.token_hex(12)
        self.started_ns = time.monotonic_ns()
        try:self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:self.boot_id = None

    def record(self, stage, started_ns, *, ended_ns=None, outcome='ok', **identity):
        ended_ns = time.monotonic_ns() if ended_ns is None else ended_ns
        if (not isinstance(stage, str) or not 0 < len(stage) <= 160
                or type(started_ns) is not int or type(ended_ns) is not int or started_ns<0 or ended_ns < started_ns
                or len(identity) > 16 or any(not isinstance(k,str) or len(k)>80 or k in ('sequence','duration_ns')
                    or type(v) not in (str,int,float,bool,type(None))
                    or isinstance(v,str) and len(v)>256
                    or type(v)is float and not math.isfinite(v) for k,v in identity.items())):
            self.invalid += 1
            return
        duration = ended_ns-started_ns
        with self.lock:
            if stage not in self.stages:
                if len(self.stages) >= 128:
                    self.dropped += 1
                    return
                self.stages[stage] = dict(count=0, total_ns=0, max_ns=0, rejected=0,
                                          samples=deque(maxlen=self.sample_count))
            stat = self.stages[stage]
            stat['count'] += 1; stat['total_ns'] += duration
            stat['max_ns'] = max(stat['max_ns'], duration)
            stat['rejected'] += outcome != 'ok'
            stat['samples'].append(duration)
            self.sequence += 1
            self.events.append(dict(sequence=self.sequence, stage=stage, started_ns=started_ns,
                                    ended_ns=ended_ns, duration_ns=duration, outcome=outcome, **identity))

    def fault(self, reason, **identity):
        """Preserve the preceding window only on an actual pipeline fault."""
        probes.fault(reason, **identity)
        with self.lock:
            self.faults.append(dict(reason=str(reason)[:256], sampled_ns=time.monotonic_ns(),
                identity=identity, events=list(self.events)[-2048:]))

    def snapshot(self, *, after=0, limit=128, monitor_history=False):
        if type(after)is not int or after<0 or type(limit)is not int or not 0<=limit<=512:
            raise ValueError('nonnegative timing cursor and limit in 0..512 required')
        with self.lock:
            stats = {k:dict(v, samples=tuple(v['samples'])) for k,v in self.stages.items()}
            retained = list(self.events)
            selected = [row for row in retained if row['sequence'] > after][:limit] if after else retained[-limit:] if limit else []
            faults = [dict(reason=f['reason'],sampled_ns=f['sampled_ns'],identity=dict(f['identity']),
                           event_count=len(f['events']),oldest_sequence=f['events'][0]['sequence']if f['events']else None,
                           events=([row for row in f['events']if row['sequence']>after][:limit]if after else
                                   f['events'][-limit:]if limit else [])) for f in self.faults]
            sequence = self.sequence
        for stat in stats.values():
            samples = sorted(stat.pop('samples'))
            stat.update(mean_ns=stat['total_ns']/stat['count'], recent_count=len(samples),
                recent_p50_ns=samples[(len(samples)-1)//2], recent_p95_ns=samples[math.ceil(len(samples)*.95)-1])
        return dict(schema=SCHEMA, instance_id=self.instance, pid=os.getpid(), boot_id=self.boot_id,
            clock='host_monotonic_ns', sampled_ns=time.monotonic_ns(), sampled_wall_ns=time.time_ns(),
            started_ns=self.started_ns, capacity=self.capacity, latest_sequence=sequence,
            oldest_sequence=retained[0]['sequence'] if retained else None,
            cursor_gap=bool(after and retained and after<retained[0]['sequence']-1),
            next_after=selected[-1]['sequence'] if selected else after,
            invalid_measurements=self.invalid, dropped_stages=self.dropped,
            percentile_scope='last bounded samples per stage; counts/mean/max since process start',
            stages=stats, events=selected, faults=faults, monitor=probes.snapshot(history=monitor_history))


timing = PipelineTiming()
