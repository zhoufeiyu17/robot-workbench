"""Byte-bounded immutable control evidence during recorder startup; CPU only."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import pickle
import queue
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from real_recording import RecordingEventQueue


class RecordingEventBudgetTest(unittest.TestCase):
    def test_byte_budget_accepts_more_than_512_events_without_mutable_aliases(self):
        pending=RecordingEventQueue(max_bytes=1024*1024)
        expected=[]
        for tick in range(4000):
            event=('feedback',{'tick':tick,'state':{'joints':[tick,.2]},'raw':{'sample_wall_ns':tick+1}},tick,1.)
            expected.append(pickle.loads(pickle.dumps(event)))
            pending.put_nowait(event)
            event[1]['state']['joints'][0]=-1
        status=pending.snapshot()
        self.assertEqual(status['depth'],4000)
        self.assertIsNone(status['capacity'])
        self.assertLessEqual(status['peak_bytes'],status['max_bytes'])
        for event in expected:
            self.assertEqual(pending.get_nowait(),event);pending.task_done()
        self.assertEqual(pending.snapshot()['queued_bytes'],0)
        self.assertEqual(pending.unfinished_tasks,0)

    def test_byte_overflow_is_atomic_and_explicit_count_limit_is_still_available(self):
        event=('rgbd',{'blob':b'x'*4096},1,1.)
        size=sys.getsizeof(pickle.dumps(event,protocol=5))+64
        pending=RecordingEventQueue(max_bytes=size)
        pending.put_nowait(event)
        before=pending.snapshot()
        with self.assertRaises(queue.Full):pending.put_nowait(event)
        self.assertEqual(pending.snapshot(),before)
        self.assertEqual(pending.get_nowait(),event);pending.task_done()
        limited=RecordingEventQueue(capacity=1)
        limited.put_nowait(event)
        with self.assertRaises(queue.Full):limited.put_nowait(event)
        self.assertEqual(limited.snapshot()['depth'],1)

    def test_concurrent_admission_cannot_overbook_last_bytes(self):
        event=('feedback',{'tick':0,'sample_wall_ns':1},1,1.)
        size=sys.getsizeof(pickle.dumps(event,protocol=5))+64
        pending=RecordingEventQueue(max_bytes=size*13)
        def submit(_):
            try:pending.put_nowait(event);return True
            except queue.Full:return False
        with ThreadPoolExecutor(max_workers=8) as workers:
            self.assertEqual(sum(workers.map(submit,range(40))),13)
        self.assertEqual(pending.snapshot()['peak_bytes'],size*13)
        self.assertEqual(pending.snapshot()['accepted_events'],13)


def c9_startup_regression(fixture,output):
    """Replay the exact frozen plan with synthetic clocks/pixels, never hardware."""
    from bisect import bisect_right
    from copy import deepcopy
    import hashlib
    import io
    import json
    import math
    import resource
    import shutil
    import threading
    import time
    from PIL import Image
    from execution_timeline import build
    from real_recording import RecordingSession,RealRecorder,verify_raw,iter_rgbd,CAMERAS,SIDES
    from real_rgbd import pair_metadata
    from robot_workbench.rgbd_channel import MemoryRgbdBudget,MemoryRgbdChannel
    fixture=Path(fixture);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    source,plan,binding=[json.loads((fixture/name).read_text()) for name in ('source.json','plan.json','hardware-binding.json')]
    if len(plan['ticks'])!=3707:raise ValueError('this regression requires the unchanged 3707-tick C9 plan')
    # One explicitly synthetic initialization tick, not reconstructed real motion.
    start=deepcopy(plan['ticks'][0]['target']);start['left_arm_joints'][0]+=.0005
    limits=deepcopy(source['initial_state']['reported_joint_limits'])
    limits.update(left_gripper=[0.,83.],right_gripper=[0.,83.])
    timeline=build(plan,current_state=start,limits=limits,environment='physical',disabled_grippers=['right'],held_arms=['right'])
    assert len(timeline['commands'])==3708
    wall=[1_900_000_000_000_000_000];origin=wall[0]+100_000_000
    released=threading.Event();entered=threading.Event();current=[]
    budget=MemoryRgbdBudget()
    channels={camera:MemoryRgbdChannel(budget=budget,clock_ns=lambda:wall[0],
        lease_guard=lambda lease:bool(current) and current[0].lease_active(lease)) for camera in CAMERAS}
    def delayed(*args,**kwargs):
        entered.set()
        if not released.wait(90):raise TimeoutError('CPU regression did not release startup')
        return RealRecorder(*args,**kwargs)
    owner=dict(owner_pid=1234,command_epoch=1,runtime_nonce='synthetic-c9-recording-regression')
    storage='segments' if shutil.which('ffmpeg') and shutil.which('ffprobe') else 'zip'
    session=RecordingSession(output/'synthetic-raw',source,plan,binding,camera_channels=channels,
        storage_format=storage,clock_ns=lambda:wall[0],environment='physical',execution_owner=owner,
        defer_execution=True,execution_instance_id='synthetic-c9-startup',preparation_active=lambda:True,
        preparation_deadline_monotonic=time.monotonic()+2,recorder_factory=delayed)
    current.append(session)
    canonical=lambda value:json.dumps(value,sort_keys=True,separators=(',',':')).encode()
    expected={name:hashlib.sha256() for name in ('command','feedback',*CAMERAS)}
    counts=dict.fromkeys(expected,0);latencies=[];legacy=queue.Queue(maxsize=512);legacy_failure=None
    duration=timeline['duration_s'];times=timeline['times']
    schedule=[(round(t*1e9),0,i) for i,t in enumerate(times)]
    schedule += [(i*10_000_000,1,i) for i in range(math.ceil(duration*100)+1)]
    schedule += [(round(i*1e9/30),2,i) for i in range(math.ceil(duration*30)+2)]
    schedule.sort()
    def pixels(index):
        return bytes((i+index)%256 for i in range(12)),b''.join(v.to_bytes(2,'little') for v in (0,1,32768,65535))
    try:
        assert session.prepared.wait(3),session.snapshot()
        assert entered.wait(3)
        session.bind_execution(timeline,execution_instance_id='synthetic-c9-startup',execution_owner=owner)
        for offset,kind,index in schedule:
            wall[0]=origin+offset;t=offset/1e9
            tick=min(len(times)-1,max(0,bisect_right(times,t+1e-10)-1))
            if kind<2:
                target=timeline['commands'][index if kind==0 else tick]['target']
                name='command' if kind==0 else 'feedback'
                fields=(dict(tick=index,target=target,send_monotonic_s=100+t,send_wall_ns=wall[0]) if kind==0 else
                    dict(tick=tick,state=target,sample_monotonic_s=100+t,sample_wall_ns=wall[0],
                        raw=dict(sample_time_s=100+t,sample_wall_ns=wall[0],
                                 qpos=[*target['left_arm_joints'],target['left_gripper'],*target['right_arm_joints'],target['right_gripper']],
                                 qvel=[0.]*14,sdk_sample_time_s=[100+t,100+t],synthetic=True)))
                before=time.perf_counter_ns();session.submit(name,**fields);latencies.append(time.perf_counter_ns()-before)
                expected[name].update(canonical(fields)+b'\n');counts[name]+=1
                if legacy_failure is None:
                    try:legacy.put_nowait((name,fields,wall[0],time.monotonic()))
                    except queue.Full:legacy_failure=dict(after_events=legacy.qsize(),elapsed_source_s=t,event=name)
            else:
                for camera,channel in channels.items():
                    meta=pair_metadata(serial=source['camera_serials'][SIDES[camera]],camera_name=SIDES[camera],
                        color_number=index,depth_number=index,color_ms=offset/1e6,depth_ms=offset/1e6,
                        color_domain='hardware_clock',depth_domain='hardware_clock',host_receive_ns=wall[0],
                        depth_scale_m=.001,intrinsics=dict(width=2,height=2,fx=100.,fy=100.,cx=1.,cy=1.))
                    meta.update(capture_instance_id='synthetic-c9-'+camera,environment='physical',simulation_only=False,
                        evidence_source='physical_rgbd',clock_domain='host_wall',source_clock_id=None,data_valid=True)
                    rgb,depth=pixels(index)
                    assert channel.write_pixels(meta,rgb,depth,width=2,height=2,sampled_token=channel.token())
                    expected[camera].update(canonical(meta)+b'\n');counts[camera]+=1
        queued=session.events.snapshot();camera_peak=budget.snapshot()
        assert queued['depth']==counts['command']+counts['feedback']>512 and not session.error
        session.finish('completed');released.set()
        assert session.done.wait(180),session.snapshot()
        assert session.error is None and session.seal_error is None,session.snapshot()
        manifest,_,_=verify_raw(session.root,environment='physical')
        assert manifest['capture_status']=='complete',manifest['validation']
        for kind in ('command','feedback'):
            actual=hashlib.sha256();seen=0
            with (session.root/(kind+'s.jsonl' if kind=='command' else 'feedback.jsonl')).open() as stream:
                for line in stream:actual.update(canonical(json.loads(line))+b'\n');seen+=1
            assert seen==counts[kind] and actual.digest()==expected[kind].digest(),kind
        for camera in CAMERAS:
            actual=hashlib.sha256();seen=0
            for _row,meta,color,depth in iter_rgbd(session.root,camera,environment='physical'):
                actual.update(canonical(meta)+b'\n')
                with Image.open(io.BytesIO(color)) as image:assert image.tobytes()==pixels(seen)[0]
                with Image.open(io.BytesIO(depth)) as image:assert list(image.getdata())==[0,1,32768,65535]
                seen+=1
            assert seen==counts[camera] and actual.digest()==expected[camera].digest(),camera
        latencies.sort();rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        result=dict(schema='c9_recording_startup_regression.v1',synthetic_only=True,motion=False,
            original_plan_ticks=3707,synthetic_runtime_ticks=3708,synthetic_duration_s=duration,
            input_sha256={name:hashlib.sha256((fixture/name).read_bytes()).hexdigest() for name in ('source.json','plan.json','hardware-binding.json')},
            storage_format=storage,legacy_512_event_queue_failure=legacy_failure,queued_before_startup=queued,
            camera_queue=camera_peak,final_event_queue=session.events.snapshot(),counts=counts,
            submit_latency_us=dict(p50=latencies[len(latencies)//2]/1000,p99=latencies[int(len(latencies)*.99)]/1000,max=max(latencies)/1000),
            process_peak_rss_bytes=rss if sys.platform=='darwin' else rss*1024,
            startup_timings=session.recorder.startup_timings,capture_status=manifest['capture_status'],
            event_pixel_depth_timestamp_metadata_exact=True,raw=str(session.root))
        (output/'regression.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        return result
    finally:
        released.set()
        if not session.done.is_set():session.finish('stopped','CPU regression cleanup');session.done.wait(30)


if __name__=='__main__':
    if len(sys.argv)==4 and sys.argv[1]=='--c9':
        import json
        print(json.dumps(c9_startup_regression(sys.argv[2],sys.argv[3]),ensure_ascii=False,indent=2))
    else:unittest.main()
