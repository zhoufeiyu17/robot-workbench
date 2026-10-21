"""Bounded GC timings; callbacks perform no IO and never scan the process heap."""
from collections import deque
import gc
import threading
import time

_pauses=deque(maxlen=64)
_slow_pauses=deque(maxlen=64)
_started={}
_control_owners=set()
_control_lock=threading.Lock()
_restore_enabled=False


def acquire_control(owner):
    """Collect only before SDK output exists; keep refcount reclamation active."""
    global _restore_enabled
    with _control_lock:
        if not _control_owners:
            _restore_enabled=gc.isenabled()
            gc.collect()
            gc.disable()
        _control_owners.add(id(owner))


def release_control(owner):
    with _control_lock:
        if id(owner) not in _control_owners:return
        _control_owners.remove(id(owner))
        if not _control_owners:
            if _restore_enabled:gc.enable()
            gc.collect()


def _record(phase, info):
    generation=info['generation']
    now=time.monotonic()
    if phase=='start':_started[generation]=now
    else:
        start=_started.pop(generation,None)
        if start is not None:
            row=dict(generation=generation,started_monotonic_s=start,
                returned_monotonic_s=now,duration_s=now-start,
                collected=info.get('collected'),uncollectable=info.get('uncollectable'))
            _pauses.append(row)
            if row['duration_s']>=.005:_slow_pauses.append(row)


def install():
    if _record not in gc.callbacks:gc.callbacks.append(_record)


def snapshot():
    rows=list(_pauses)
    # ponytail: cycles wait for SDK exit; monitor RSS before long-duration promotion.
    return dict(scope='business_process_gc_only',automatic_enabled=gc.isenabled(),
        control_owners=len(_control_owners),policy='cyclic_collection_only_without_sdk_output',
        pending_counts=list(gc.get_count()),pauses=rows,slow_pauses=list(_slow_pauses),
        max_pause_s=max((r['duration_s'] for r in rows),default=None),
        over_100ms=sum(r['duration_s']>.1 for r in rows))
