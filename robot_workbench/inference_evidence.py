"""Bounded single-thread raw inference evidence; never writes on the model caller."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
import re
import threading
import time

import numpy as np


class InferenceEvidence:
    def __init__(self, root, capacity=8):
        self.root=Path(root)
        self.writer=ThreadPoolExecutor(max_workers=1,thread_name_prefix='inference-evidence')
        self.slots=threading.BoundedSemaphore(capacity)
        self.lock=threading.Lock()
        self.pending=0;self.completed=0;self.error=None;self.closed=False

    def snapshot(self):
        with self.lock:
            return dict(pending=self.pending,completed=self.completed,error=self.error,closed=self.closed,
                        status='failed'if self.error else'pending'if self.pending else'complete',
                        durability='background_write_completion; not fsync')

    def _submit(self, request_id, operation, value):
        if not isinstance(request_id,str)or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',request_id):
            raise ValueError('bounded evidence request identity required')
        with self.lock:
            if self.closed or self.error:raise ValueError(self.error or 'inference evidence writer closed')
            if not self.slots.acquire(blocking=False):
                self.error='inference evidence queue full; no silent data loss'
                raise ValueError(self.error)
            self.pending+=1
            try:future=self.writer.submit(self._write,self.root/request_id,operation,value,time.monotonic_ns())
            except BaseException:
                self.pending-=1;self.slots.release();raise
        def finished(task):
            error=task.exception()
            with self.lock:
                self.pending-=1;self.completed+=1
                if error is not None:self.error=self.error or str(error)
            self.slots.release()
        future.add_done_callback(finished)
        return future

    def input(self, request_id, state, images, metadata):
        # Freeze before handing ownership to a slower writer. No disk calls here.
        arrays={'state14':np.array(state,copy=True),**{k:np.array(v,copy=True)for k,v in images.items()}}
        for value in arrays.values():value.flags.writeable=False
        return self._submit(request_id,'input',(arrays,deepcopy(metadata)))

    def output(self, request_id, response, *, chunk, timing=None):
        arrays={'action14':np.array(response['action14'],copy=True)}
        arrays['action14'].flags.writeable=False
        values={'audit.json':deepcopy(response['audit'])}
        if timing is not None:values['pipeline-timing.json']=deepcopy(timing)
        if chunk:values['chunk.json']=deepcopy({k:v for k,v in response.items()if k not in('action14','audit')})
        return self._submit(request_id,'output',(arrays,values))

    def failure(self, request_id, error):
        value={'error':str(error)}
        if getattr(error,'guard_decision',None) is not None:value['guard_decision']=deepcopy(error.guard_decision)
        return self._submit(request_id,'failure',value)

    @staticmethod
    def _json(path, value):
        from .owner_inference import plain
        temporary=path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(json.dumps(plain(value),ensure_ascii=False,allow_nan=False,indent=2)+'\n')
        temporary.replace(path)

    @staticmethod
    def _npz(path, arrays):
        temporary=path.with_suffix(path.suffix+'.tmp')
        with temporary.open('wb')as stream:np.savez(stream,**arrays)
        temporary.replace(path)

    def _write(self, directory, operation, value, submitted_ns):
        started=time.monotonic_ns()
        if operation=='input':
            directory.mkdir(parents=True,exist_ok=False)
            arrays,metadata=value
            self._npz(directory/'input.npz',arrays)
            self._json(directory/'request.json',metadata)
        elif operation=='output':
            arrays,values=value
            self._npz(directory/'output.npz',arrays)
            for name,row in values.items():self._json(directory/name,row)
        else:self._json(directory/'prediction-error.json',value)
        self._json(directory/(operation+'-write.json'),dict(submitted_monotonic_ns=submitted_ns,
            started_monotonic_ns=started,completed_monotonic_ns=time.monotonic_ns(),operation=operation))

    def close(self):
        with self.lock:self.closed=True
        self.writer.shutdown(wait=True)  # Drain outside control/Stop and inference calls.
