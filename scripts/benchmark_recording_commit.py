#!/usr/bin/env python3
"""CPU-only three-camera commit/queue replay; never acquires sensors or a GPU.

Historical mode verifies one sealed input and replays its actual RGB8/depth16
pixels and source timestamps, optionally from a verified read-only decoded cache.
The separate synthetic mode labels its generated depth explicitly; it cannot
qualify captured data. Both trials share the same input and fixed CPU affinity.
"""
import argparse
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
import mmap
import os
import resource
from pathlib import Path
import shutil
import sys
import threading
import time
import zipfile

for variable in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[variable]='1'
os.environ['CUDA_VISIBLE_DEVICES']=''
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from rgbd_segments import SegmentWriter,iter_segment,recover
from robot_workbench.rgbd_channel import MemoryRgbdBudget,MemoryRgbdChannel

CAMERAS=('base_0','left_wrist_0','right_wrist_0')
TRANSPORT_FIELDS={'recording_run_id','recording_deadline_wall_ns','recording_duration_s','recorded_frames','recording_enabled','recording_error'}


class HistoricalInputs:
    """Read one actual pair at a time; retain source pixels and every source clock."""
    def __init__(self,root,expected_sha,frames):
        self.root=Path(root).resolve(strict=True);raw=(self.root/'manifest.json').read_bytes()
        if hashlib.sha256(raw).hexdigest()!=expected_sha:raise ValueError('historical manifest hash changed')
        self.manifest=json.loads(raw)
        if self.manifest.get('capture_status')!='complete' or self.manifest.get('validation',{}).get('status')!='passed':
            raise ValueError('complete sealed history required')
        self.rows={name:[json.loads(line) for line in (self.root/(name+'-pairs.jsonl')).read_text().splitlines()] for name in CAMERAS}
        if any(len(values)!=self.manifest['camera_frames'][name] or len(values)<frames for name,values in self.rows.items()):raise ValueError('historical index count mismatch')
        self.owner=self.manifest['execution_owner'];self.frames=frames
        self.cache=None
        self.first_wall=min(int(values[0]['host_receive_ns']) for values in self.rows.values())
        self.first_source=min(values[0]['source_time_ns'] for values in self.rows.values())
        for name,item in self.manifest['files'].items():
            path=self.root/name
            if Path(name).is_absolute() or '..' in Path(name).parts or not path.is_relative_to(self.root) or path.is_symlink():raise ValueError('unsafe historical artifact')
            digest=hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
            if path.stat().st_size!=item['size_bytes'] or digest.hexdigest()!=item['sha256']:raise ValueError('historical file integrity failed: '+name)
        self.provenance=dict(root=str(self.root),manifest_sha256=expected_sha,source_count=self.manifest['camera_frames'],
            replay_count_per_camera=frames,original_files_verified=len(self.manifest['files']),
            timestamps='original source and host timestamps retained exactly; only replay scheduling uses source cadence',
            pacing='original source-time differences; finite historical input, no invented post-terminal frames',
            historical_wall_duration_s=(max(int(v[-1]['host_receive_ns']) for v in self.rows.values())-self.first_wall)/1e9)

    def frame(self,name,index):
        import numpy as np
        from PIL import Image
        row=self.rows[name][index]
        if self.cache is not None:
            size=640*480*5;start=index*size;data=self.cache[name][start:start+size]
            return {k:deepcopy(v) for k,v in row.items() if k!='path'},data[:640*480*3],data[640*480*3:]
        with zipfile.ZipFile(self.root/row['path']) as archive:
            metadata=json.loads(archive.read('metadata.json'));values=[]
            for kind,mode,dtype in [('color.png','RGB','uint8'),('depth.png',None,'<u2')]:
                with Image.open(io.BytesIO(archive.read(kind))) as image:
                    if image.size!=(640,480) or mode and image.mode!=mode:raise ValueError('original RGB8/depth16 640x480 required')
                    values.append(np.asarray(image,dtype=dtype).tobytes())
        if any(metadata.get(k)!=v for k,v in row.items() if k!='path'):raise ValueError('ZIP metadata differs from sealed index')
        return metadata,*values

    def delay(self,index):return (min(self.rows[n][index]['source_time_ns'] for n in CAMERAS)-self.first_source)/1e9

    def endpoint(self,name):return self.rows[name][self.frames-1]['source_time_ns']

    def prepare_cache(self,path):
        path.mkdir(parents=True,exist_ok=False);files={}
        for name in CAMERAS:
            target=path/(name+'.rgbd');digest=hashlib.sha256()
            with target.open('xb') as stream:
                for index in range(len(self.rows[name])):
                    _,rgb,depth=self.frame(name,index)
                    stream.write(rgb);stream.write(depth);digest.update(rgb);digest.update(depth)
                stream.flush();os.fsync(stream.fileno())
            target.chmod(0o444);files[target.name]=dict(size_bytes=target.stat().st_size,sha256=digest.hexdigest())
        save(path/'manifest.json',dict(source=self.provenance['manifest_sha256'],counts=self.manifest['camera_frames'],
            layout='640x480 RGB8 bytes followed by 640x480 little-endian depth16 bytes, frame-major',files=files,
            frame_map={name:[dict(index=i,byte_offset=i*640*480*5,size_bytes=640*480*5,
                **{key:row[key] for key in ('pair_id','source_time_ns','host_receive_ns')}) for i,row in enumerate(rows)] for name,rows in self.rows.items()},
            preparation_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
        (path/'manifest.json').chmod(0o444)

    def attach_cache(self,path):
        receipt=json.loads((path/'manifest.json').read_text())
        if receipt['source']!=self.provenance['manifest_sha256'] or receipt['counts']!=self.manifest['camera_frames']:
            raise ValueError('decoded cache differs from original sealed input')
        self.cache={}
        for name in CAMERAS:
            target=path/(name+'.rgbd');expected=receipt['files'][target.name];digest=hashlib.sha256()
            with target.open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
                if target.stat().st_size!=len(self.rows[name])*640*480*5 or expected!=dict(size_bytes=target.stat().st_size,sha256=digest.hexdigest()):
                    raise ValueError('decoded RGB-D cache integrity failed')
                self.cache[name]=mmap.mmap(stream.fileno(),0,access=mmap.ACCESS_READ)
        self.provenance['decoded_cache']=dict(path=str(path),verified=True,original_decoded_pixels_exact=True,
            resident_policy='read-only mmap; current frame copied into unchanged 1GiB recording queue')


def save(path,value):
    temporary=path.with_suffix(path.suffix+'.partial')
    temporary.write_text(json.dumps(value,allow_nan=False,indent=2)+'\n');temporary.replace(path)


def inputs(seed,rgb_paths):
    import numpy as np
    rng=np.random.default_rng(seed);values={};provenance={}
    yy,xx=np.indices((480,640))
    for slot,camera in enumerate(CAMERAS):
        if rgb_paths:
            from PIL import Image
            path=Path(rgb_paths[slot]).resolve(strict=True)
            with Image.open(path) as image:
                if image.size!=(640,480) or image.mode!='RGB':raise ValueError('existing RGB inputs must already be RGB8 640x480; no resizing')
                base=np.asarray(image).copy()
            provenance[camera]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        else:
            base=((xx[...,None]//2+yy[...,None]//3+np.array([11,83,157]))%256).astype('uint8')
            base^=rng.integers(0,32,size=base.shape,dtype='uint8')
            provenance[camera]=dict(kind='generated_spatial_texture',seed=seed)
        frames=[]
        for phase in range(8):
            rgb=np.roll(base,phase*3,axis=1).tobytes()
            depth=((xx*71+yy*13+phase*17+rng.integers(0,32,size=(480,640)))%65536).astype('<u2')
            depth[(xx+yy+phase)%31==0]=0
            depth[0,:4]=[0,1,32768,65535]
            frames.append((rgb,depth.tobytes()))
        values[camera]=frames
    digest=hashlib.sha256()
    for camera in CAMERAS:
        for rgb,depth in values[camera]:digest.update(rgb);digest.update(depth)
    return values,dict(rgb=provenance,depth='generated uint16 full-range spatial depth plus noise/holes',
        input_sha256=digest.hexdigest(),source_clock='generated 30Hz fixture; no captured timestamps relabelled')


def trial(output,frames,payloads,*,workers,cutoff,ffmpeg,ffprobe,tail_timeout,max_bytes,interrupt_after=None):
    output.mkdir(parents=True,exist_ok=False)
    history=isinstance(payloads,HistoricalInputs)
    budget=MemoryRgbdBudget();owner=payloads.owner if history else dict(owner_pid=os.getpid(),command_epoch=1,runtime_nonce='cpu-commit-benchmark')
    now=payloads.first_wall if history else time.time_ns()
    channels={name:MemoryRgbdChannel(lease_guard=lambda _:True,budget=budget,clock_ns=lambda:now) for name in CAMERAS}
    pool=ThreadPoolExecutor(max_workers=workers,thread_name_prefix='rgbd-commit')
    writers={name:SegmentWriter(output/'segments',name,commit_executor=pool,ffmpeg=ffmpeg,ffprobe=ffprobe) for name in CAMERAS}
    tokens={name:format(index+1,'032x') for index,name in enumerate(CAMERAS)}
    for name,channel in channels.items():channel.begin(dict(run_id=tokens[name],start_wall_ns=now-1,
        deadline_wall_ns=now+round((frames/30+tail_timeout+30)*1e9),environment='fake',execution_owner=owner,execution_instance_id='cpu-commit-benchmark'))
    stop=threading.Event();ended=threading.Event();producer_error=[];lateness=[];supply_finished=[None]
    counts={name:0 for name in CAMERAS};supplied={name:0 for name in CAMERAS};started=time.monotonic();endpoint=1_000_000_000+(frames-1)*1_000_000_000//30
    stage=['supply_and_write'];memory_peaks={};monitor_done=threading.Event()
    def sample_memory():
        pending=[os.getpid()];seen=set();rows=[]
        while pending:
            pid=pending.pop()
            if pid in seen:continue
            seen.add(pid)
            try:
                fields={line.split(':',1)[0]:int(line.split()[1]) for line in Path(f'/proc/{pid}/status').read_text().splitlines() if line.split(':',1)[0] in ('VmRSS','RssAnon','RssFile','Threads')}
                rows.append((pid,fields))
                for children in Path(f'/proc/{pid}/task').glob('*/children'):pending.extend(map(int,children.read_text().split()))
            except (OSError,ValueError):pass
        values=dict(parent_rss_kib=next((row.get('VmRSS',0) for pid,row in rows if pid==os.getpid()),0),
            process_tree_rss_kib=sum(row.get('VmRSS',0) for _,row in rows),process_tree_anon_kib=sum(row.get('RssAnon',0) for _,row in rows),
            process_tree_file_kib=sum(row.get('RssFile',0) for _,row in rows),processes=len(rows),os_threads=sum(row.get('Threads',0) for _,row in rows))
        result=memory_peaks.setdefault(stage[0],{})
        for name,value in values.items():result[name]=max(result.get(name,0),value)
    def monitor():
        while not monitor_done.wait(.25):sample_memory()
    monitor_thread=threading.Thread(target=monitor,daemon=True);monitor_thread.start()
    def produce():
        index=0
        try:
            while not stop.is_set() and (not history or index<frames):
                target=started+(payloads.delay(index) if history else index/30)
                if stop.wait(max(0.,target-time.monotonic())):break
                lateness.append(max(0.,time.monotonic()-target))
                for name,channel in channels.items():
                    if history:meta,rgb,depth=payloads.frame(name,index)
                    else:
                        rgb,depth=payloads[name][index%8]
                        meta=dict(owner,environment='fake',frame_number=index,depth_frame_number=index,
                        pair_id=name+':'+str(index),host_receive_ns=time.time_ns(),source_time_ns=1_000_000_000+index*1_000_000_000//30,
                        source_clock_id='cpu-generated-input',clock_domain='simulation',physics_step=index*8,data_valid=True,
                        evidence_source='generated_cpu_storage_fixture')
                    channel.write_pixels(meta,rgb,depth,width=640,height=480,sampled_token=tokens[name]);supplied[name]+=1
                if index>=frames-1:
                    if cutoff:
                        for name,channel in channels.items():channel.stop_after(tokens[name],payloads.endpoint(name) if history else endpoint)
                    supply_finished[0]=time.monotonic();ended.set()
                index+=1
        except BaseException as error:producer_error.append(str(error));stop.set()
    producer=threading.Thread(target=produce,daemon=True);producer.start()
    failure=None;cut=False;last_report=started;peak_processes=0;peak_threads=0;submitted=sealed=None
    try:
        while min(counts.values())<frames:
            if producer_error:raise RuntimeError(producer_error[0])
            if time.monotonic()-started>frames/30+tail_timeout:raise TimeoutError('bounded recording drain timed out')
            if cutoff and ended.is_set() and not cut:
                for name,channel in channels.items():channel.stop_after(tokens[name],payloads.endpoint(name) if history else endpoint)
                cut=True
            consumed=False
            for name,channel in channels.items():
                for _ in range(4):
                    if counts[name]>=frames:break
                    frame=channel.peek(tokens[name])
                    if frame is None:break
                    writers[name].add_pixels(frame['metadata'],frame['rgb'],frame['depth'],640,480)
                    channel.ack(tokens[name],frame['metadata']['pair_id']);counts[name]+=1;consumed=True
            if time.monotonic()-last_report>=1:
                child_pids=[]
                for writer in writers.values():
                    with writer._lock:child_pids.extend(child.process.pid for child in writer._owned_children if child.process.poll() is None)
                thread_count=0
                for pid in child_pids:
                    try:thread_count+=int(next(line.split()[1] for line in Path(f'/proc/{pid}/status').read_text().splitlines() if line.startswith('Threads:')))
                    except (OSError,StopIteration):pass
                peak_processes=max(peak_processes,len(child_pids));peak_threads=max(peak_threads,thread_count)
                size=sum(path.stat().st_size for path in output.rglob('*') if path.is_file())
                if size>max_bytes:raise RuntimeError('per-trial output byte budget exceeded')
                save(output/'progress.json',dict(elapsed_s=time.monotonic()-started,counts=counts,supplied=supplied,
                    memory=budget.snapshot(),codec_processes=len(child_pids),codec_os_threads=thread_count,output_bytes=size))
                last_report=time.monotonic()
            if not consumed:time.sleep(.001)
            if interrupt_after is not None and min(counts.values())>=interrupt_after:
                raise RuntimeError('intentional CPU writer interruption after bounded committed prefix')
        submitted=time.monotonic();stage[0]='seal';stop.set();producer.join(2)
        for writer in writers.values():writer.finish()
        sealed=time.monotonic()
    except Exception as error:
        failure=type(error).__name__+': '+str(error)
    finally:
        stop.set();producer.join(2)
        channel_receipts={name:channel.snapshot() for name,channel in channels.items()}
        for name,channel in channels.items():channel.end(tokens[name])
        for writer in writers.values():writer.abort()
        pool.shutdown(wait=True,cancel_futures=True)
    stage[0]='verify_committed_prefix';verify_started=time.monotonic();verified={};verification_error=None
    try:
        for name in CAMERAS:
            count=0
            for path in sorted((output/'segments'/name).glob('*.commit.json')):
                for meta,rgb,depth in iter_segment(output/'segments',str(path.relative_to(output/'segments')),ffmpeg=ffmpeg,ffprobe=ffprobe,png=False):
                    if history:
                        expected_meta,a,b=payloads.frame(name,count);expected=(a,b)
                        metadata_exact={k:v for k,v in meta.items() if k not in TRANSPORT_FIELDS}=={k:v for k,v in expected_meta.items() if k not in TRANSPORT_FIELDS}
                    else:
                        expected=payloads[name][count%8]
                        metadata_exact=meta['frame_number']==count and meta['source_time_ns']==1_000_000_000+count*1_000_000_000//30
                    if (rgb,depth)!=expected or not metadata_exact:raise ValueError('decoded pixels, order or source time changed')
                    count+=1
            verified[name]=count
    except Exception as error:verification_error=str(error);failure=failure or 'committed prefix verification failed: '+str(error)
    finally:sample_memory();monitor_done.set();monitor_thread.join(2)
    recovery={name:{key:value for key,value in recover(output/'segments',name,ffprobe=ffprobe).items() if key!='segments'}
        for name in CAMERAS} if interrupt_after is not None else None
    ordered=sorted(lateness)
    result=dict(status='failed' if failure else 'passed',error=failure,commit_workers=workers,cutoff=cutoff,
        frames_requested=frames,frames_written=counts,frames_verified=verified,published=supplied,
        source_duration_s=payloads.delay(frames-1) if history else (frames-1)/30,write_wall_s=(submitted or time.monotonic())-started,
        tail_wall_s=None if submitted is None else max(0.,submitted-started-(frames-1)/30),
        supply_complete_wall_s=None if supply_finished[0] is None else supply_finished[0]-started,
        tail_after_last_received_s=None if submitted is None or supply_finished[0] is None else submitted-supply_finished[0],
        seal_wall_s=None if sealed is None else sealed-submitted,verify_wall_s=time.monotonic()-verify_started,
        supply_lateness_p95_s=ordered[int((len(ordered)-1)*.95)] if ordered else None,
        supply_lateness_max_s=max(ordered) if ordered else None,memory=budget.snapshot(),channels=channel_receipts,
        codec_process_peak=peak_processes,codec_os_thread_peak=peak_threads,
        memory_peak_by_stage=memory_peaks,memory_sampling_interval_s=.25,
        interrupted_prefix=dict(verification_error=verification_error,verified_frames=verified,
            intentional_interruption=interrupt_after,committed_only=True,partial_files_retained=True,
            complete_episode=not failure and len(verified)==3 and all(count==frames for count in verified.values())),
        recovery=recovery,
        timings={name:writer.timings for name,writer in writers.items()},
        complete=not failure and len(verified)==3 and all(count==frames for count in verified.values()),
        decoded_pixels_exact=verification_error is None,source_indices_exact=verification_error is None,
        original_metadata_exact_except_transport=history and verification_error is None,gpu_used=False,sensor_acquisition=False)
    save(output/'result.json',result);return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--frames',type=int,default=1201)
    parser.add_argument('--seed',type=int,default=20261017);parser.add_argument('--rgb-files',nargs=3)
    parser.add_argument('--historical-root',type=Path);parser.add_argument('--manifest-sha256')
    parser.add_argument('--decoded-cache',type=Path);parser.add_argument('--prepare-cache',action='store_true')
    parser.add_argument('--interrupt-after',type=int)
    parser.add_argument('--ffmpeg',default='ffmpeg');parser.add_argument('--ffprobe',default='ffprobe')
    parser.add_argument('--tail-timeout',type=float,default=60.);parser.add_argument('--max-output-bytes',type=int,default=8*1024**3)
    parser.add_argument('--dry-run',action='store_true');args=parser.parse_args()
    if not 1<=args.frames<=1801 or not 1<=args.tail_timeout<=180 or args.max_output_bytes<=0:raise ValueError('bounded benchmark required')
    if args.interrupt_after is not None and not 120<args.interrupt_after<args.frames:raise ValueError('interruption must follow a full committed segment and precede input end')
    if not args.output.is_absolute() or args.output.exists():raise ValueError('new absolute output directory required')
    if not shutil.which(args.ffmpeg) or not shutil.which(args.ffprobe):raise ValueError('real ffmpeg and ffprobe are required; no simulated codec fallback')
    affinity=sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None
    if affinity is not None and len(affinity)>4:raise ValueError('launch this CPU benchmark with taskset on at most four permitted cores')
    if args.historical_root:
        if args.rgb_files or not args.manifest_sha256:raise ValueError('one pinned historical input required')
        payloads=HistoricalInputs(args.historical_root,args.manifest_sha256,args.frames);provenance=payloads.provenance
    else:payloads,provenance=inputs(args.seed,args.rgb_files)
    receipt=dict(schema='recording_commit_benchmark.v1',pid=os.getpid(),input=provenance,frames=args.frames,
        dimensions=[640,480],fps=30,lossless_rgb8_depth16=True,affinity=affinity,
        cpu_only=True,hardware_qualification=False,codec_threads_per_process=1,
        budget_note='six RGB/depth encoders plus bounded probes; CPU affinity <=4, not a claim of four OS threads')
    if args.dry_run:print(json.dumps(dict(receipt,status='dry_run_passed',input_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)));return
    if args.prepare_cache:
        if not args.decoded_cache or not args.decoded_cache.is_absolute() or not isinstance(payloads,HistoricalInputs):raise ValueError('new absolute historical cache required')
        payloads.prepare_cache(args.decoded_cache);print(json.dumps(dict(status='decoded_cache_prepared',path=str(args.decoded_cache))));return
    if args.decoded_cache:payloads.attach_cache(args.decoded_cache)
    args.output.mkdir(parents=True,exist_ok=False);save(args.output/'run.json',receipt)
    results={}
    for name,workers,cutoff in (('baseline-serial',1,False),('bounded-parallel',3,True)):
        results[name]=trial(args.output/name,args.frames,payloads,workers=workers,cutoff=cutoff,
            ffmpeg=args.ffmpeg,ffprobe=args.ffprobe,tail_timeout=args.tail_timeout,max_bytes=args.max_output_bytes//2,interrupt_after=args.interrupt_after)
        save(args.output/'receipt.json',dict(receipt,results=results))
    print(json.dumps(dict(output=str(args.output),results={name:{key:row[key] for key in ('status','complete','tail_wall_s','seal_wall_s')} for name,row in results.items()})))


if __name__=='__main__':main()
