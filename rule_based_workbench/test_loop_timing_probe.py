"""Deterministic CPU-only probe checks; no SDK or robot connection."""
from unittest.mock import patch
from real_control import LoopTimingProbe, classes, load_calibration
from types import SimpleNamespace
import threading
import sys


def main(calibration_root):
    clock=[10.,1.]
    rows=[]
    emit=lambda kind,**fields: rows.append((kind,fields))
    with patch('time.monotonic',lambda:clock[0]),patch('time.thread_time',lambda:clock[1]):
        probe=LoopTimingProbe();probe.begin(emit)
        calls=[]
        def slow_read():
            calls.append('read');clock[0]+=.04;clock[1]+=.002
            return 'sample'
        assert probe.call('feedback_read',slow_read)=='sample'
        probe.begin(emit)
        assert not rows
        for _ in range(5):
            clock[0]+=.01;clock[1]+=.001;probe.begin(emit)
        assert len(rows)==1 and calls==['read']
        event=rows[0][1];span=event['before'][-1][4][0]
        assert span[0]=='feedback_read' and abs(span[2]-.04)<1e-8 and abs(span[3]-.002)<1e-8
        assert len(event['after'])==5 and event['trigger_loop']==1
        for _ in range(2000):
            clock[0]+=.01;probe.begin(emit)
        assert len(rows)==1 and len(probe.history)==12
        for _ in range(100):probe.call('tiny',lambda:None)
        assert len(probe.current[3])==48 and probe.dropped_spans==52
        try:probe.call('fault',lambda:(_ for _ in ()).throw(ValueError('unchanged')))
        except ValueError as exc:assert str(exc)=='unchanged'
        else:raise AssertionError('probe swallowed exception')
    # Simulated GC notifications exercise attribution without forcing collection.
    with patch('time.monotonic',lambda:clock[0]),patch('time.thread_time',lambda:clock[1]):
        probe=LoopTimingProbe()
        probe.gc_callback('start',{'generation':2})
        probe.gc_callback('stop',{'generation':2})
        assert probe.gc_started is None
        probe.begin(emit)
        for generation in range(3):
            probe.gc_callback('start',{'generation':generation})
            clock[0]+=.028;clock[1]+=.0277
            probe.gc_callback('stop',{'generation':generation})
            span=probe.current[3][-1]
            assert span[0]=='gc_gen_'+str(generation)
            assert abs(span[2]-.028)<1e-8 and abs(span[3]-.0277)<1e-8
        count=len(probe.current[3])
        def foreign_gc():
            probe.gc_callback('start',{'generation':2})
            probe.gc_callback('stop',{'generation':2})
        thread=threading.Thread(target=foreign_gc);thread.start();thread.join()
        assert len(probe.current[3])==count and probe.gc_started is None
        while len(probe.current[3])<48:probe.call('tiny',lambda:None)
        probe.gc_callback('start',{'generation':2});probe.gc_callback('stop',{'generation':2})
        assert len(probe.current[3])==48 and probe.dropped_spans==1
        probe.begin(emit)
        probe.flush(emit)
        assert any(s[0]=='gc_gen_2' for frame in rows[-1][1]['before'] for s in frame[4])
    # Actual shared Owner.tick path: passive fake read/cache only, absolutely no send.
    ref=load_calibration(calibration_root)
    Owner,_,_=classes(ref)
    from test_real_control import Takeover,State
    backend=ref.SimulatedBackend()
    probe=LoopTimingProbe();probe.begin(emit)
    owner=Owner(backend,SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State),timing_probe=probe)
    before=list(backend.events)
    owner.tick()
    assert backend.events==before, 'probe/passive tick issued a backend operation'
    stages={span[0] for span in probe.current[3]}
    assert {'owner_tick','feedback_read','cache_state','cache_state_base'}<=stages,stages
    count=len(probe.current[3])
    thread=threading.Thread(target=lambda:probe.call('http',lambda:None));thread.start();thread.join()
    assert len(probe.current[3])==count, 'foreign thread corrupted owner timing'
    # A blocked CPU initializer must leave the sole feedback owner running.
    from test_frozen_executor import fixture
    import execution_timeline
    source,plan=fixture(.001);plan['stages']=[dict(name='initial_open',start_tick=0)]
    Owner,sdk,_=classes(ref)
    started,resume=threading.Event(),threading.Event()
    original_initialize=execution_timeline.initialize
    def compile_blocked(*args,**kwargs):
        started.set()
        if not resume.wait(2):raise AssertionError('initialization test was not released')
        return original_initialize(*args,**kwargs)
    with patch('time.monotonic',lambda:clock[0]):
        backend=ref.SimulatedBackend(sdk(plan['ticks'][0]['target']))
        owner=Owner(backend,SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State))
        for _ in range(42):owner.tick();clock[0]+=.01
        preview=owner.submit(dict(action='preview_frozen',plan=plan,source=source))
        owner.submit(dict(action='confirm',token=preview['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        with patch.object(execution_timeline,'initialize',side_effect=compile_blocked):
            try:
                for _ in range(40):
                    owner.tick();clock[0]+=.01
                    if owner.runner.runtime_started:break
                assert started.wait(1)
                before=list(backend.events)
                for _ in range(20):owner.tick();clock[0]+=.01
                state=owner.snapshot()
                assert state['feedback_healthy'] and state['sample_age_s']<.02
                assert owner.runner.status=='VERIFYING' and backend.events==before
            finally:resume.set()
            assert owner.runner.runtime_ready.wait(2)
            owner.tick()
            assert owner.runner.status=='RUNNING',owner.snapshot()
            assert owner.runner.runtime['control_hz']==30
    print('loop timing: bounded spans, passive no-send and off-thread initialization with fresh owner feedback passed')

if __name__=='__main__':main(sys.argv[1])
