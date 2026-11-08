"""Opt-in native ROS load diagnostic; synthetic RGB/state, no robot or GPU."""
from pathlib import Path
from collections import deque
import argparse
import hashlib
import json
import multiprocessing
import os
import resource
import shutil
import sys
import tempfile
import threading
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]


def wait_for_seal(session,timeout_s,*,close_sources=lambda:None,progress=lambda state:None):
    """Observer deadline is not a recording outcome; retain the original writer."""
    deadline=None;closed=False
    while True:
        state=session.snapshot()
        if state.get('ingress_closed')and not closed:
            close_sources();closed=True;deadline=time.monotonic()+timeout_s
        if session.done.is_set():
            if not state.get('sealed')or state.get('writer_exited')is not True:
                raise RuntimeError('recording terminal outcome unconfirmed; inspect original session')
            return state
        progress(state)
        remaining=.5 if deadline is None else deadline-time.monotonic()
        if remaining<=0:raise TimeoutError('recording seal still pending; original resident writer preserved')
        session.done.wait(min(.5,remaining))


def subscriber(sources,stop,output,name,timeout_s):
    from robot_workbench.ros_data import RosNode
    from robot_workbench.ros_inputs import RosInputs
    ros=RosNode(name,namespace='/wb_load',domain_id=127);counts={row['stream_id']:0 for row in sources}
    ages={key:deque(maxlen=4096)for key in counts}
    try:
        def received(ref,value):
            counts[ref.stream_id]+=1;ages[ref.stream_id].append((time.monotonic_ns()-ref.source_time_ns)/1e6)
        inputs=RosInputs(ros,sources,on_sample=received,max_samples=4,max_bytes=4*1024**2)
        stop.wait(timeout_s)
        output.put(dict(name=name,pid=os.getpid(),counts=counts,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                        errors=list(ros.errors),cache_bytes={key:value.bytes for key,value in inputs.caches.items()},
                        source_qos=inputs.reliability,age_sample_limit=4096,age_ms={key:dict(p50=float(np.percentile(value,50)),p99=float(np.percentile(value,99)),maximum=max(value))
                            for key,value in ages.items()if value}))
    finally:ros.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--seal-timeout-s',type=float,default=1800,
                        help='post-ingress-closure seal observation deadline; expiry preserves the writer and reports pending')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--depth',action='store_true',help='include original aligned16UC1 depth streams')
    parser.add_argument('--width',type=int,default=640)
    parser.add_argument('--height',type=int,default=480)
    parser.add_argument('--trace-control',action='store_true',help='trace the first4096 control events across publication, DDS callback and writer admission')
    parser.add_argument('--control-observer',action='store_true',help='add a direct control subscriber to distinguish the effect of reader fanout')
    args=parser.parse_args()
    if not 5<=args.seconds<=600:parser.error('bounded load duration must be5..600 seconds')
    if not 0<args.seal_timeout_s<=7200:parser.error('seal observation deadline must be positive and at most7200 seconds')
    if not 2<=args.width<=640 or not 2<=args.height<=480 or args.width%2 or args.height%2:
        parser.error('even dimensions within2..640 by2..480 required')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':parser.error('isolated localhost ROS required')
    if args.control_observer and not args.trace_control:parser.error('--control-observer requires --trace-control')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    os.environ['CUDA_VISIBLE_DEVICES']='-1'
    from sensor_msgs.msg import Image
    from robot_workbench.ros_data import RosNode
    from robot_workbench.ros_driver import RosDriverPublisher
    from robot_workbench.ros_recording import capture_source
    from robot_workbench.recording_process import CaptureSession
    from test_recording_process import start_recording_fixture,stop_recording_fixture
    report={};session=None;threads=[];children=[];fixture=None;ros=None;driver=None
    directory=Path(tempfile.mkdtemp(prefix="ros-load-"))
    trace_path=directory/'control-ingress-trace.json';published_control=[];observed_control=[]
    stop=threading.Event();ctx=multiprocessing.get_context('spawn');child_stop=ctx.Event();output=ctx.Queue()
    try:
        ros=RosNode('load_source',namespace='/wb_load',domain_id=127)
        driver=RosDriverPublisher(ros,device_id='cpu_load',instance_id='owner',clock_id='host_monotonic',
            state_topic='/wb_load/state',event_topic='/wb_load/control')
        if args.trace_control:
            publisher=driver.events
            class TracedPublisher:
                def publish(self,message):
                    publisher.publish(message)
                    if len(published_control)<4096:published_control.append(message.reference.sequence)
            driver.events=TracedPublisher()
            def observe_control(message):
                if len(observed_control)<4096:observed_control.append(message.reference.sequence)
            if args.control_observer:ros.subscribe('/wb_load/control',ros.types.DataEvent,observe_control,depth=256)
        def state():return dict(qpos=[0.]*14,qvel=[0.]*14,sample_time_s=time.monotonic(),sample_wall_ns=time.time_ns())
        driver.publish_state(state());driver.flush()
        sources=[dict(driver.source_values[0])];publishers=[];depth_publishers=[];rgb_pixels={};depth_pixels={};declarations=[]
        for index,name in enumerate(('base_0','left_wrist_0','right_wrist_0')):
            sources.append(dict(device_id='cpu_load',stream_id=name,source_instance_id='camera',source_clock_id='host_monotonic',
                topic='/wb_load/'+name,message_type='sensor_msgs/msg/Image',kind='image',units='rgb8',coordinate_frame=name,
                nominal_hz=30.,available=True,metadata=dict(width=args.width,height=args.height,serial=name,environment='fake')))
            pixels=np.random.default_rng(index).integers(0,256,(args.height,args.width,3),dtype=np.uint8).tobytes()
            rgb_pixels[name]=pixels
            image=Image(height=args.height,width=args.width,encoding='rgb8',step=args.width*3,data=pixels)
            publishers.append((ros.publisher('/wb_load/'+name,Image),image))
            if args.depth:
                from robot_workbench.ros_data import descriptor_message
                sources[-1]['metadata']['depth_stream_id']=name+'_depth'
                raw=np.random.default_rng(index+10).integers(0,65536,(args.height,args.width),dtype=np.uint16).astype('<u2').tobytes()
                depth_pixels[name]=raw
                value=dict(sources[-1],stream_id=name+'_depth',topic='/wb_load/'+name+'_depth',units='depth16le',
                    metadata=dict(width=args.width,height=args.height,serial=name,environment='fake',pixel_format='depth16le',
                        depth_scale_m=.001,depth_aligned_to='color'))
                declared=ros.publisher('sources/'+name+'_depth',ros.types.SourceDescriptor,retained=True,depth=1)
                declared.publish(descriptor_message(ros,value));declarations.append(declared)
                depth_publishers.append((ros.publisher(value['topic'],Image),Image(height=args.height,width=args.width,encoding='16UC1',step=args.width*2,data=raw)))
        for name in ('input_subscriber','model_input_subscriber'):
            child=ctx.Process(target=subscriber,args=(sources,child_stop,output,name,args.seconds+240));child.start();children.append(child)
        calls=deque(maxlen=10000);event_calls=deque(maxlen=10000);ticks=[0];frames=[0];image_stamps=[]
        def clocked(period,call):
            due=time.monotonic()
            while not stop.is_set():
                call();due+=period;stop.wait(max(0.,due-time.monotonic()))
        def sample():
            value=state();start=time.perf_counter_ns();driver.publish_state(value);calls.append(time.perf_counter_ns()-start);ticks[0]+=1
            start=time.perf_counter_ns()
            for phase in ('feedback','command_attempt','command_returned','guard'):
                driver.event(phase,{'state_ref':driver.reference('state')},run_id='load',
                    source_time_ns=round(value['sample_time_s']*1e9))
            event_calls.append((time.perf_counter_ns()-start)/4)
        def images():
            stamp=time.monotonic_ns()
            for publisher,image in [*publishers,*depth_publishers]:
                image.header.stamp.sec=stamp//10**9;image.header.stamp.nanosec=stamp%10**9;publisher.publish(image)
            image_stamps.append(stamp)
            frames[0]+=1
        for period,call in ((.01,sample),(1/30,images)):
            thread=threading.Thread(target=clocked,args=(period,call));thread.start();threads.append(thread)
        extra_env=None
        if args.trace_control:
            wrapper=directory/'traced-writer.py'
            wrapper.write_text(f'''#!{sys.executable}
import json,os,sys
from pathlib import Path
if sys.argv[-1]!='capture':os.execv(sys.executable,[sys.executable,*sys.argv[1:]])
from robot_workbench import recording_process as recording
from robot_workbench.ros_data import RosNode,BoundedQueue
trace={{'received':[],'admitted':[]}}
subscribe=RosNode.subscribe;put=BoundedQueue.put_nowait
def traced_subscribe(node,topic,message_type,callback,**options):
    if topic=='/wb_load/control':
        original=callback
        def callback(message):
            if len(trace['received'])<4096:trace['received'].append(message.reference.sequence)
            return original(message)
    return subscribe(node,topic,message_type,callback,**options)
def traced_put(queue,item):
    result=put(queue,item)
    ref=item[1][0] if isinstance(item[1],tuple) else None
    if getattr(ref,'stream_id',None)=='control' and len(trace['admitted'])<4096:trace['admitted'].append(ref.sequence)
    return result
RosNode.subscribe=traced_subscribe;BoundedQueue.put_nowait=traced_put
try:recording._capture_worker()
finally:Path({str(trace_path)!r}).write_text(json.dumps(trace)+'\\n')
''');wrapper.chmod(0o700);extra_env={'WORKBENCH_CPU_PYTHON':str(wrapper)}
        fixture=start_recording_fixture(extra_env)
        session=CaptureSession.create(Path(directory)/'capture',request_id='load',purpose_profile='data_required',
            source_descriptors=[capture_source(r,namespace='/wb_load',domain_id=127)for r in [*sources,driver.source_values[1]]])
        prepared=session.prepare();session.bind(run_id='load',business_kind='inference',policy_hash='a'*64,
            execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='owner',command_epoch=0),health_source=prepared['health_source']);session.start()
        started=time.monotonic();before=(ticks[0],frames[0]);snapshots=[];recorder_rss=[]
        while time.monotonic()-started<args.seconds:
            snapshot=session.snapshot();snapshots.append({'data_queue':snapshot.get('data_queue')})
            try:
                status=Path('/proc/'+str(snapshot['worker_pid'])+'/status').read_text()
                recorder_rss.append(int(next(line.split()[1]for line in status.splitlines()if line.startswith('VmRSS:'))))
            except (OSError,KeyError,StopIteration):pass
            if len(snapshots)%10==0:
                progress=dict(elapsed_s=time.monotonic()-started,state_samples=ticks[0]-before[0],frames_per_camera=frames[0]-before[1],
                    recorder_rss_kib=recorder_rss[-1]if recorder_rss else None,queue=snapshot.get('data_queue'),error=snapshot.get('error'))
                args.output.with_suffix('.progress.json').write_text(json.dumps(progress)+'\n')
            time.sleep(.5)
        ended=time.monotonic();produced=(ticks[0]-before[0],frames[0]-before[1])
        endpoints={row['stream_id']:dict(source_clock_id=row['source_clock_id'],source_time=ended)for row in sources}
        with driver.lock:final_control=driver.sequences['control']
        session.finish({'control':final_control},dict(status='completed'),source_endpoints=endpoints)
        report.update(duration_s=ended-started,width=args.width,height=args.height,depth_enabled=args.depth)
        def seal_progress(state):
            args.output.with_suffix('.progress.json').write_text(json.dumps(dict(phase='sealing',
                elapsed_s=time.monotonic()-started,ingress_closed=state.get('ingress_closed'),
                state=state.get('state'),worker_pid=state.get('worker_pid'),writer_exited=state.get('writer_exited'),
                capture_directory=str(session.directory)))+'\n')
        final=wait_for_seal(session,args.seal_timeout_s,
            close_sources=lambda:(stop.set(),child_stop.set()),progress=seal_progress)
        sealed=final['sealed'];manifest=json.loads((session.directory/'manifest.json').read_text())
        coverage={}
        for name in ('base_0','left_wrist_0','right_wrist_0'):
            stamps=[json.loads(line)['payload']['metadata']['ros_reference']['source_time_ns']
                    for line in(session.directory/(name+'.jsonl')).read_text().splitlines()]
            expected={stamp for stamp in image_stamps if stamps and stamps[0]<=stamp<=stamps[-1]}
            coverage[name]=dict(received=len(stamps),expected=len(expected),missing=len(expected-set(stamps)))
        sequences=[json.loads(line)['payload']['reference']['sequence']
            for line in(session.directory/'control.jsonl').read_text().splitlines()]
        control_coverage=dict(received=len(sequences),expected=sequences[-1]-sequences[0]+1,
            missing=sequences[-1]-sequences[0]+1-len(set(sequences)),final_target=final_control,last=sequences[-1])
        pixel_checks={}
        for kind,original in (('rgb',rgb_pixels),('depth',depth_pixels)):
            pixel_checks[kind]={}
            for name,pixels in original.items():
                segments=list((session.directory/'camera'/name).glob('*.commit.json'))
                for segment in segments:
                    value=json.loads(segment.read_text());digest=hashlib.sha256()
                    for _ in range(value['frame_count']):digest.update(pixels)
                    if value['decoded_pixel_sha256'].get(kind)!=digest.hexdigest():raise RuntimeError('original '+kind+' pixels differ')
                pixel_checks[kind][name]=dict(verified_segments=len(segments),original_pixels_verified=bool(segments))
        report=dict(duration_s=ended-started,width=args.width,height=args.height,enqueue_sample_limit=10000,
            source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()for name in
                ('tests/test_ros_load.py','robot_workbench/ros_driver.py','robot_workbench/ros_data.py','robot_workbench/ros_inputs.py','robot_workbench/ros_recording.py','robot_workbench/recording_process.py')},
            state_hz=produced[0]/(ended-started),camera_hz=produced[1]/(ended-started),
            control_hz=produced[0]*4/(ended-started),control_coverage=control_coverage,
            depth_enabled=args.depth,depth_checks=pixel_checks['depth'],rgb_checks=pixel_checks['rgb'],
            raw_rgbd_bytes_s=produced[1]*3*args.width*args.height*(5 if args.depth else 3)/(ended-started),
            rgb_raw_bytes_s=produced[1]*3*args.width*args.height*3/(ended-started),source_dropped=driver.dropped,source_error=driver.error,
            state_enqueue_us={key:float(np.percentile(calls,q)/1000)for key,q in [('p50',50),('p95',95),('p99',99)]},
            event_enqueue_us={key:float(np.percentile(event_calls,q)/1000)for key,q in [('p50',50),('p95',95),('p99',99)]},
            source_queue=driver.queue.snapshot(),recorder_queues=[s.get('data_queue')for s in snapshots],
            max_source_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            sampled_recorder_max_rss_kib=max(recorder_rss,default=None),recorder_rss_samples=len(recorder_rss),recorder_rss_kib=recorder_rss,manifest=manifest,
            sealed=sealed,recording_state=session.snapshot())
        report['published_frame_coverage']=coverage
        if args.trace_control:
            trace=json.loads(trace_path.read_text())if trace_path.exists()else{}
            report['control_trace']=dict(sample_limit=4096,observer_enabled=args.control_observer,published=published_control,observer=observed_control,
                **trace,recorded=sequences[:4096])
        stop.set()
        for thread in threads:thread.join(5)
        child_stop.set();report['subscribers']=[output.get(timeout=10)for _ in children]
        for child in children:child.join(10)
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
        if (manifest.get('recording_outcome')!='complete' or driver.dropped
                or any(row['errors']or not all(row['counts'].values())for row in report['subscribers'])):
            raise RuntimeError('native load did not preserve declared source coverage; see report')
        if any(row['missing'] for row in coverage.values()):raise RuntimeError('published frame coverage has holes; see report')
        if control_coverage['missing']or control_coverage['last']<final_control:raise RuntimeError('published control event coverage has holes; see report')
        report['outcome']='complete';args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({key:report[key]for key in ('duration_s','state_hz','camera_hz','rgb_raw_bytes_s','state_enqueue_us','max_source_rss_kib','sealed')}))
    except Exception as error:
        report["error"]=str(error)
        report['outcome']='pending'if isinstance(error,TimeoutError)and session is not None and not session.done.is_set()else'failed'
        if session is not None:
            report["recording_state"]=session.snapshot()
            report["capture_directory"]=str(session.directory)
            log=session.directory/"worker.log"
            if log.exists():report["writer_log_tail"]=log.read_text(errors="replace")[-12000:]
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+"\n")
        raise
    finally:
        stop.set();child_stop.set()
        for thread in threads:thread.join(5)
        for child in children:
            child.join(5)
            if child.is_alive():child.terminate();child.join(5)
        if session is not None and session.done.is_set() and session.process is not None and session.process.poll()is not None and not (session.directory/"manifest.json").exists():
            session.recover();session.done.wait(30)
        if session is not None and not session.done.is_set():
            try:session.close(timeout=0 if session.terminal is not None else 30)
            except TimeoutError:report['retained_capture_directory']=str(session.directory)
        if driver is not None:driver.close()
        if ros is not None:ros.close()
        if fixture is not None:
            from robot_workbench.recording_process import recording_call
            descriptor=Path(fixture[1].name)/'service/service.json'
            if recording_call(descriptor,'inspect')['can_stop']:stop_recording_fixture(fixture)
            else:report['retained_service_descriptor']=str(descriptor)
        if report.get('error'):args.output.write_text(json.dumps(report,indent=2)+'\n')
        if not report.get('error'):shutil.rmtree(directory)


if __name__=='__main__':main()
