"""Bounded numeric ingress evidence, with one optional background file write.

No pixels, leases or control credentials enter this recorder. Diagnostics never
change a capture decision, wait for storage in the capture loop, or retry SDK IO.
"""
from collections import deque
from bisect import bisect_right
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import hashlib
import math
import os
from pathlib import Path
import tempfile
import time


SCHEMA='capture_ingress_diagnostics.v1'
STAGES=('wait','align','convert','publish')
BINS_NS=(1_000_000,5_000_000,10_000_000,20_000_000,33_333_333,50_000_000,100_000_000,1_000_000_000)
JOURNAL_LIMIT=16*1024**2
JOURNAL_QUEUE=128


def unsupported():
    return dict(schema=SCHEMA,supported=False,evidence_type='simulation_sample',
        journal=dict(supported=False,reason='simulation has no physical SDK ingress stages'),
        reason='SDK frameset and align stages do not apply to the PhysX source')


class CaptureDiagnostics:
    def __init__(self,output,instance):
        self.output=Path(output);self.instance=instance
        self.recent=deque(maxlen=32);self.events=deque(maxlen=4)
        self.attempts=self.accepted=self.rejected=self.skipped_writes=0
        self.last_raw=self.last_published=None
        self.pending_event=None;self.future=None;self.write_error=None
        self.closed=False;self.active_stage='initializing'
        self.timings={name:dict(count=0,total_ns=0,maximum_ns=0,histogram=[0]*(len(BINS_NS)+1)) for name in STAGES}
        self.event_count=0;self.journal_queue=deque(maxlen=JOURNAL_QUEUE)
        self.journal_queue_dropped=self.journal_file_omitted=0
        self.journal_bytes=self.journal_records=0;self.journal_full=False
        self.journal_committed=(0,0,None)
        self.journal_submitted=0
        self.journal_path=self.output/('capture-events-'+hashlib.sha256(instance.encode()).hexdigest()[:24]+'.jsonl')
        self.journal_stream=None;self.journal_hasher=hashlib.sha256();self.journal_sha256=None
        self.writer=ThreadPoolExecutor(max_workers=1,thread_name_prefix='capture-diagnostics')
        self._enqueue(dict(record_type='capture_started',host_wall_ns=time.time_ns(),
            host_monotonic_ns=time.monotonic_ns(),event_scope='pair rejection/counter gaps and associated stage timings',
            summary_interval_attempts=900,queue_limit=JOURNAL_QUEUE,file_limit_bytes=JOURNAL_LIMIT))

    def journal(self):
        committed_bytes,committed_records,digest=self.journal_committed
        return dict(schema='capture_ingress_journal.v1',capture_instance_id=self.instance,
            path=str(self.journal_path),committed_bytes=committed_bytes,
            committed_records=committed_records,committed_prefix_sha256=digest,
            submitted_records=self.journal_submitted,
            queued_records=len(self.journal_queue),queue_dropped=self.journal_queue_dropped,
            file_omitted=self.journal_file_omitted,full=self.journal_full,limit_bytes=JOURNAL_LIMIT,
            scope='bounded diagnostic history; gaps and write failures never alter capture acceptance')

    def _enqueue(self,value):
        self.journal_submitted+=1
        if len(self.journal_queue)==JOURNAL_QUEUE:self.journal_queue_dropped+=1
        self.journal_queue.append(dict(schema='capture_ingress_event.v1',capture_instance_id=self.instance,
            record_sequence=self.journal_submitted,**value))

    def stage(self,name,elapsed_ns):
        if name not in self.timings or type(elapsed_ns) is not int or elapsed_ns<0:
            raise ValueError('known capture stage and nonnegative duration required')
        row=self.timings[name];row['count']+=1;row['total_ns']+=elapsed_ns
        row['maximum_ns']=max(row['maximum_ns'],elapsed_ns)
        row['histogram'][bisect_right(BINS_NS,elapsed_ns)]+=1
        if name in ('convert','publish') and self.recent:
            self.recent[-1].setdefault('stage_ns',{})[name]=elapsed_ns
            if 'event_index' in self.recent[-1]:
                row=self.recent[-1]
                self._enqueue(dict(record_type='stage_completed',attempt=row['attempt'],event_index=row['event_index'],
                    host_receive_ns=row['host_receive_ns'],stage=name,elapsed_ns=elapsed_ns))
                if name=='publish':self.persist()

    def attempt(self,raw,aligned,accepted,decision,*,host_receive_ns,stage_ns=None):
        def numeric(value):
            return {key:(value.get(key) if type(value.get(key)) in (int,float) and math.isfinite(value[key]) else None)
                for key in ('rgb_number','depth_number','rgb_device_ms','depth_device_ms')}
        raw,aligned=numeric(raw),numeric(aligned)
        self.attempts+=1;self.accepted+=bool(accepted);self.rejected+=not accepted
        current=tuple(aligned.get(key) for key in ('rgb_number','depth_number'))
        raw_current=tuple(raw.get(key) for key in ('rgb_number','depth_number'))
        def jump(before,after):
            return before is not None and all(type(x) is int for x in (*before,*after)) and any(b-a>1 or b<a for a,b in zip(before,after))
        event=(jump(self.last_raw,raw_current) or accepted and jump(self.last_published,current) or not accepted)
        row=dict(attempt=self.attempts,raw=deepcopy(raw),aligned=deepcopy(aligned),
            accepted=bool(accepted),decision=decision,host_receive_ns=host_receive_ns,
            previous_published=list(self.last_published) if self.last_published else None,stage_ns=dict(stage_ns or {}))
        self.recent.append(row);self.last_raw=raw_current
        if accepted:self.last_published=current
        if event:
            self.event_count+=1;row['event_index']=self.event_count
            self.events.append(deepcopy(row));self.pending_event=dict(schema=SCHEMA,
                capture_instance_id=self.instance,event=deepcopy(row),recent_attempts=deepcopy(list(self.recent)))
            self._enqueue(dict(record_type='pair_decision',**deepcopy(row)))
            self.persist()
        if self.attempts==1 or self.attempts%900==0:
            self._enqueue(dict(record_type='capture_summary',attempt=self.attempts,accepted=self.accepted,
                rejected=self.rejected,event_count=self.event_count,host_receive_ns=host_receive_ns,
                journal=self.journal()))
            self.persist()

    def take_event(self):
        value,self.pending_event=self.pending_event,None
        return value

    def summary(self):
        return dict(schema=SCHEMA,supported=True,evidence_type='physical_sdk_ingress',
            capture_instance_id=self.instance,attempts=self.attempts,accepted=self.accepted,rejected=self.rejected,
            unfinished_stage=self.active_stage,
            maximum_stage_ns={name:row['maximum_ns'] for name,row in self.timings.items()},
            diagnostic_write_error=self.write_error,diagnostic_writes_skipped=self.skipped_writes,
            journal=self.journal())

    def snapshot(self):
        return dict(self.summary(),stage_timings=deepcopy(self.timings),
            histogram_upper_bounds_ns=[*BINS_NS,None],
            recent_attempts=deepcopy(list(self.recent)),recent_events=deepcopy(list(self.events)),
            retention=dict(attempts=32,events=4,journal_queue=JOURNAL_QUEUE,pixels=False),attribution='not_determined')

    def _write(self,value):
        records=value.pop('_journal_records',[])
        for row in records:
            encoded=(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n').encode()
            if self.journal_full or self.journal_bytes+len(encoded)>JOURNAL_LIMIT:
                self.journal_full=True;self.journal_file_omitted+=1;continue
            if self.journal_stream is None:self.journal_stream=self.journal_path.open('xb',buffering=0)
            if self.journal_stream.write(encoded)!=len(encoded):raise OSError('short diagnostic journal write')
            self.journal_hasher.update(encoded);self.journal_sha256=self.journal_hasher.hexdigest()
            self.journal_bytes+=len(encoded);self.journal_records+=1
            self.journal_committed=(self.journal_bytes,self.journal_records,self.journal_sha256)
        value['journal']=self.journal()
        with tempfile.NamedTemporaryFile(mode='w',dir=self.output,prefix='.ingress-',suffix='.json',delete=False) as stream:
            temporary=stream.name;json.dump(value,stream,allow_nan=False,indent=2)
        os.replace(temporary,self.output/'capture-diagnostics.json')

    def persist(self):
        if self.future is not None:
            if not self.future.done():self.skipped_writes+=1;return
            error=self.future.exception()
            if error is not None:self.write_error=str(error)[:500]
        if self.write_error:return
        value=self.snapshot();value['_journal_records']=list(self.journal_queue);self.journal_queue.clear()
        self.future=self.writer.submit(self._write,value)

    def close(self,reason='close_requested'):
        if self.closed:return
        self.closed=True
        self._enqueue(dict(record_type='capture_ended',reason=reason,attempt=self.attempts,accepted=self.accepted,
            rejected=self.rejected,event_count=self.event_count,host_wall_ns=time.time_ns(),
            host_monotonic_ns=time.monotonic_ns(),journal=self.journal()))
        try:
            if self.future is not None:
                self.future.result()
            if not self.write_error:
                value=self.snapshot();value['_journal_records']=list(self.journal_queue);self.journal_queue.clear()
                self._write(value)
        except (OSError,ValueError,TypeError) as error:
            self.write_error=str(error)[:500]
        finally:
            self.writer.shutdown(wait=True)
            if self.journal_stream is not None:self.journal_stream.close()
