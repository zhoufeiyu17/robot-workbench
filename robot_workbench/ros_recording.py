"""ROS subscriptions inside the resident recorder's owned writer process.

Lifecycle still uses CaptureSession's existing private RPC. Source data never
travels through the business process. The existing lossless segment format and
manifest reader remain the storage boundary.
"""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import select
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

from .ros_data import RosNode, StreamRef, json_text, json_value, source_descriptor,descriptor_message,RECORDING_HEALTH_FIELDS
from .ros_inputs import RosInputs
from .ros_inference import reconstruct_input


def capture_source(source, *, namespace, domain_id):
    source=source_descriptor(source)
    kinds={'image':'video','state':'state','model_input':'model','model_result':'model','event':'event'}
    if source['kind']not in kinds:raise ValueError('unsupported ROS capture source kind')
    return dict(stream_id=source['stream_id'],kind=kinds[source['kind']],source_instance_id=source['source_instance_id'],
        source_clock_id=source['source_clock_id'],units=source['units'],coordinate_frame=source['coordinate_frame'],
        nominal_hz=source['nominal_hz'] or None,required=source['kind']in('image','state'),
        source_ref=dict(kind='ros',source=source,namespace=namespace,domain_id=domain_id))


def _plain(value):
    if hasattr(value,'get_fields_and_field_types'):
        return {name:_plain(getattr(value,name))for name in value.get_fields_and_field_types()}
    if isinstance(value,(str,int,float,bool))or value is None:return value
    if isinstance(value,dict):return {k:_plain(v)for k,v in value.items()}
    return [_plain(v)for v in value]


def _recording_health_source(root,spec,identity):
    """Reuse the Run and actual writer birth identity; no second UUID."""
    plane=spec['source_descriptors'][0]['source_ref']
    writer={key:identity[key]for key in ('pid','start_ticks','boot_id')}
    instance=':'.join(str(writer[key])for key in ('pid','start_ticks','boot_id'))
    stream=root.name.replace('-','_')+'_health'
    topic=plane['namespace']+'/recording/'+stream+'/writer_'+str(writer['pid'])+'_'+str(writer['start_ticks'])
    node=RosNode('capture_health_'+str(writer['pid']),namespace=plane['namespace'],domain_id=plane['domain_id'],localhost_only=False)
    try:
        publisher=node.publisher(topic,node.types.DataEvent,retained=True,depth=1)
        deadline=time.monotonic()+2.
        while True:
            found=node.node.get_publishers_info_by_topic(topic)
            if found:break
            if time.monotonic()>=deadline:raise TimeoutError('original recording health publisher not discovered')
            time.sleep(.01)
        if len(found)!=1 or found[0].node_name!=node.node.get_name():raise ValueError('one original recording health publisher required')
        source=source_descriptor(dict(device_id=plane['source']['device_id'],stream_id=stream,source_instance_id=instance,
            source_clock_id='host_monotonic',topic=topic,message_type='workbench_interfaces/msg/DataEvent',kind='event',
            units='capture_health',coordinate_frame='recording',nominal_hz=5.,available=True,
            metadata=dict(capture_session_id=spec['capture_session_id'],writer_identity=writer,publisher_gid=bytes(found[0].endpoint_gid).hex())))
        declared=node.publisher('sources/'+stream,node.types.SourceDescriptor,retained=True,depth=1)
        declared.publish(descriptor_message(node,source))
        return node,dict(kind='ros',source=source,namespace=plane['namespace'],domain_id=plane['domain_id']),publisher,declared
    except BaseException:node.close();raise


def capture_worker(root, spec, remaining=b'', *, recover=False):
    from .recording_process import _CaptureEvents, _capture_file, _encode, _seal_capture, _archive_capture_payload
    from .inference_trace import write_json
    from rgbd_segments import SegmentWriter
    root=Path(root)
    from .managed_services import process_identity
    writer_identity=process_identity(os.getpid())
    if recover:
        manifest=_seal_capture(root,spec,'recovered after recorder exit; no motion replay')
        state=dict(state=manifest['recording_outcome'],prepared=True,ready=False,done=True,
            capture_session_id=spec['capture_session_id'],worker_pid=__import__('os').getpid(),
            manifest_ref='manifest.json',seal_revision=manifest['seal_revision'],streams=manifest['streams'],writer_identity=writer_identity)
        write_json(root/'status.json',state);sys.stdout.write(_encode(state));sys.stdout.flush();return
    refs=[r['source_ref']for r in spec['source_descriptors']]
    if any(r['kind']!='ros'for r in refs)or len({(r['namespace'],r['domain_id'])for r in refs})!=1:
        raise ValueError('one explicit ROS domain/namespace required; mixed source transports rejected')
    sources={r['source']['stream_id']:r['source']for r in refs}
    ros=RosNode(root.name.replace('-','_'),namespace=refs[0]['namespace'],domain_id=refs[0]['domain_id'],localhost_only=False)
    events=_CaptureEvents(spec['resource_budget']['event_queue_bytes'])
    lock=threading.RLock();errors={};active=False;binding=None;terminal=None;ending_at=None
    activity=None;activity_prefix_ns=0;observed_last={};written_groups={};prefilled_until={}
    received={k:0 for k in sources};flushed=dict(received);first={};last={};files={};segments={};model_results=set();artifact_index=None
    recorded_models={}
    file_locks={k:threading.Lock()for k in sources}
    encoders=ThreadPoolExecutor(max_workers=3,thread_name_prefix='ros-capture-camera')
    pending_images={}
    journal=ThreadPoolExecutor(max_workers=1,thread_name_prefix='ros-capture-journal')
    pending_flush=None;artifact_lock=threading.Lock()
    emit_lock=threading.RLock();health_node=health_source=health_publisher=health_declared=health_thread=None
    health_closed=threading.Event();health_changed=threading.Event();health_failed=threading.Event()
    health_sequence=0;control_signature=None;source_seen=set()
    def fault(stream,reason):
        with lock:errors.setdefault(stream,str(reason))
    def flush_files(status):
        counts={};descriptors=[]
        for stream in list(files):
            with file_locks[stream]:
                files[stream].flush();counts[stream]=received[stream];descriptors.append(files[stream].fileno())
        with artifact_lock:
            artifact_fd=None
            if artifact_index is not None:artifact_index.flush();artifact_fd=artifact_index.fileno()
        # No producer/file lock is held while the kernel drains dirty pages.
        # The returned count covers only the prefix sampled before this fsync.
        if artifact_fd is not None:os.fsync(artifact_fd)
        for fd in descriptors:os.fsync(fd)
        write_json(root/'status.json',status)
        return counts
    def collect_image(stream,wait=False):
        future=pending_images.get(stream)
        if future is None or not wait and not future.done():return
        try:future.result(timeout=30.)
        except Exception as error:fault(stream,error)
        finally:pending_images.pop(stream,None)
    state=dict(state='preparing',prepared=False,ready=False,done=False,ingress_closed=False,capture_session_id=spec['capture_session_id'],
        worker_pid=os.getpid(),writer_identity=writer_identity,streams={},source_errors=errors,binding=None)
    def admit(ref,value,group=None,prefill=False):
        source=sources[ref.stream_id];selected=None
        if source['kind']=='image':size=len(value['rgb'])+len(value.get('depth')or b'')+4096
        elif source['kind']=='state':size=len(json.dumps(value))+4096
        else:
            from rclpy.serialization import serialize_message
            wire_size=len(serialize_message(value))
            if wire_size>spec['resource_budget']['message_bytes']:
                errors[ref.stream_id]='ROS event exceeds the bound message size';return
            size=wire_size*4+4096
            if source['kind']=='model_input':
                try:selected=inputs.resolve_observation(value.observation)
                except (LookupError,ValueError):pass  # Exact journal/cache lookup still runs when dequeued.
                if selected is not None:
                    # Pin the exact originals before slow preprocessing imports
                    # can let the live ring evict them; charge the existing queue.
                    size+=sum(len(frame['rgb'])+len(frame.get('depth')or b'')+4096 for frame in selected[1].values())
                    size+=len(json.dumps(selected[0]))+4096
        try:events.put_nowait((size,(ref,value,group,prefill,selected)))
        except queue.Full:errors.setdefault(ref.stream_id,'ROS recorder queue overflow; source data missing')
    def enqueue(ref,value):
        with lock:
            source=sources[ref.stream_id];old=observed_last.get(ref.stream_id)
            if old is None or ref.sequence>old.sequence:observed_last[ref.stream_id]=ref
            if source['kind']in('image','state')and ref.stream_id not in source_seen:
                source_seen.add(ref.stream_id)
                # Source readiness is a transition, not a periodic metric.
                # Wake the existing health publisher; never emit on the DDS callback.
                health_changed.set()
            if not active:return
            group=None;lifecycle=False
            if activity is not None:
                previous_group=activity.current['request_id']if activity.current else None
                if ref.stream_id=='control':
                    if old and ref.sequence>old.sequence+1:fault('control','activity control source sequence gap')
                    lifecycle=value.event in ('inference_mode_transition','inference_finish','inference_episode_stopped','stop','reference_cancelled')
                    data=json_value(value.payload_json)if lifecycle else {}
                    if value.event=='inference_mode_transition'and data.get('runtime_nonce')!=source['source_instance_id']:
                        raise ValueError('recording mode receipt belongs to another Owner')
                    opened=activity.observe(value.event,data,ref,value.run_id)
                    if previous_group is not None and (activity.current is None or activity.current['request_id']!=previous_group):
                        events.put_nowait((0,(None,None,None,False,None)))
                    if opened:
                        # Retain only the bounded input-reference prefix. Export
                        # excludes these evidence frames from effective video time.
                        for raw_source in raw:
                            stream=raw_source['stream_id'];clock=raw_source['source_clock_id']
                            start=opened['start_wall_ns']if clock=='host_wall'else opened['start_source_ns']
                            for saved_ref,saved in inputs.caches[stream].history():
                                if saved_ref.source_time_ns<start-activity_prefix_ns:continue
                                try:saved=inputs.rgbd(saved_ref)if raw_source['kind']=='image'else saved
                                except (LookupError,ValueError):continue
                                prefilled_until[stream]=max(prefilled_until.get(stream,0),saved_ref.sequence)
                                admit(saved_ref,saved,opened['request_id'],True)
                group=activity.current['request_id']if activity.current else previous_group if lifecycle else None
                if source['kind']in('image','state'):
                    group=activity.sample_group(ref.source_clock_id,ref.source_time_ns)
                    if group is None:return
                elif activity.current is None and not lifecycle and source['kind']not in ('model_input','model_result'):return
            admit(ref,value,group)
            if activity is not None and activity.current is None and source['kind']=='image':
                # Delayed transport can deliver an active tail after HOLD.
                # Keep it by source time, then commit that paused tail too.
                events.put_nowait((0,(None,None,None,False,None)))
    raw=[s for s in sources.values()if s['kind']in('image','state')]
    inputs=RosInputs(ros,raw,on_sample=enqueue,max_samples=16,subscription_depth=16,include_depth=True)
    def message(stream,value):
        ref=StreamRef.from_message(value.reference);source=sources[stream]
        if any(getattr(ref,k)!=source[k]for k in('stream_id','source_instance_id','source_clock_id')):
            with lock:errors.setdefault(stream,'ROS source identity changed')
            return
        # An event source has one global sequence, including late receipts for
        # older Runs. Keep its full bound window and original run_id; filtering
        # those events would create gaps and an unreachable global watermark.
        if binding is not None and (source['kind']=='event' or value.run_id==binding['run_id']):
            enqueue(ref,value)
    for stream,source in sources.items():
        if source['kind']in('image','state'):continue
        kind={'model_input':'ModelInput','model_result':'ModelResult','event':'DataEvent'}.get(source['kind'])
        if kind is None:raise ValueError('unknown ROS recording message')
        ros.subscribe(source['topic'],getattr(ros.types,kind),lambda msg,s=stream:message(s,msg),
            depth=max(1,spec['resource_budget']['event_queue_bytes']//spec['resource_budget']['message_bytes']))
    def emit(*,persist=True):
        nonlocal pending_flush,health_sequence,control_signature
        with emit_lock:
            if pending_flush is not None and pending_flush.done():
                try:flushed.update(pending_flush.result())
                except Exception as error:fault('journal',error)
                pending_flush=None
            if not state['done'] and not state['ingress_closed']:
                committed=dict(flushed)
                for stream,writer in list(segments.items()):
                    committed[stream]=min(flushed[stream],sum(s['frame_count']for s in writer.segments))
                missing=[s['stream_id']for s in raw if not inputs.caches[s['stream_id']].samples
                    or s['stream_id']in inputs.depth_for and s['stream_id']not in inputs.last_pairs]
                if ros.errors:fault('ros_callback',ros.errors[0])
                for stream,count in list(inputs.gaps.items()):
                    if count:fault(stream,'original RGB-D/dependency samples missing before bounded cache eviction')
                for key,cache in inputs.caches.items():
                    if cache.error:fault(key,cache.error)
                with lock:
                    prepared=not missing or spec['purpose_profile']=='infer_debug' or bool(errors)
                    state.update(prepared=prepared,ready=not missing or spec['purpose_profile']=='infer_debug',
                        source_qos=dict(inputs.reliability),
                        degraded=bool(errors or missing),missing_streams=missing,source_errors=dict(errors),binding=binding,
                        available_streams=[k for k in sources if k not in missing],streams={k:dict(received=v,committed=committed[k],
                            last_sequence=v,source_sequence=last[k].sequence if k in last else None)for k,v in received.items()},
                        data_queue=events.snapshot(),heartbeat_monotonic_s=time.monotonic())
                    if activity is not None:state['recording_activity']=activity.summary()
                    if state['state']=='preparing'and prepared:state['state']='prepared'
                    if errors and spec['purpose_profile']=='data_required':state.update(ready=False,error='; '.join(errors.values()))
                    snapshot=deepcopy(state)
            else:
                with lock:
                    state['heartbeat_monotonic_s']=time.monotonic();snapshot=deepcopy(state)
            health_sequence+=1
            ref=StreamRef(health_source['source']['stream_id'],health_source['source']['source_instance_id'],
                'host_monotonic',round(snapshot['heartbeat_monotonic_s']*1e9),health_sequence)
            payload={key:value for key,value in snapshot.items()if key in RECORDING_HEALTH_FIELDS}
            health_publisher.publish(health_node.types.DataEvent(reference=ref.message(),
                run_id=(binding or{}).get('run_id',''),epoch=0,event='recording_health',payload_json=json_text(payload)))
            signature=(snapshot['state'],snapshot['prepared'],snapshot['done'],snapshot['ingress_closed'])
            if signature!=control_signature:
                # This pipe acknowledges lifecycle transitions. Live health is
                # published once on ROS, not copied to each RPC consumer.
                control={key:value for key,value in snapshot.items()if key not in RECORDING_HEALTH_FIELDS
                    or key in ('state','prepared','ready')}
                if snapshot['done']:control=snapshot
                sys.stdout.write(_encode(control));sys.stdout.flush();control_signature=signature
            if persist and pending_flush is None:pending_flush=journal.submit(flush_files,snapshot)
    def health_loop():
        while not health_closed.is_set():
            health_changed.wait(.2);health_changed.clear()
            if health_closed.is_set():return
            try:emit()
            except Exception as error:
                fault('health',error);health_failed.set();return
    def append(ref,payload):
        stream=ref.stream_id
        with file_locks[stream]:
            if stream not in files:files[stream]=(root/(stream+'.jsonl')).open('x')
            row=dict(sequence=received[stream]+1,source_time=ref.source_time_ns/1e9,received_monotonic_s=time.monotonic(),
                recorded_wall_ns=time.time_ns(),payload=payload)
            files[stream].write(_encode(row))
            if not received[stream]:first[stream]=ref
            received[stream]+=1;last[stream]=ref
    class RecordedInputs:
        def resolve_observation(self,observation,*,timeout=0.):
            try:return inputs.resolve_observation(observation,timeout=timeout)
            except ValueError as error:
                if 'referenced ROS sample was evicted'not in str(error):raise
                reference=StreamRef.from_message(observation.state);stream=reference.stream_id
                if sources.get(stream,{}).get('kind')!='state'or stream not in files:raise
                # ponytail: rare evicted-state lookup scans the journal; use a
                # sparse disk index if long Runs make this scan costly.
                saved=None
                with file_locks[stream]:
                    files[stream].flush()
                    with (root/(stream+'.jsonl')).open()as journal_file:
                        while line:=journal_file.readline(spec['resource_budget']['message_bytes']+1):
                            if not line.endswith('\n'):raise ValueError('bounded original state journal row required')
                            payload=json.loads(line)['payload']
                            if payload['reference']==reference.__dict__:saved=payload['data'];break
                if saved is None:raise
                if len(observation.frames)!=len(observation.camera_roles)or len(set(observation.camera_roles))!=3:
                    raise ValueError('observation camera role mapping is incomplete')
                frames={role:inputs.resolve(ref,timeout=timeout)for role,ref in zip(observation.camera_roles,observation.frames)}
                return saved,frames
    recorded_inputs=RecordedInputs()
    def write(ref,value,group=None,prefill=False,selected=None):
        nonlocal artifact_index
        source=sources[ref.stream_id];previous=last.get(ref.stream_id)
        model_key=model_digest=None
        if source['kind']in('model_input','model_result'):
            if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',value.request_id):raise ValueError('invalid recorded model request identity')
            plain=_plain(value);actions=plain.pop('actions',None)
            digest=hashlib.sha256(json_text(plain).encode())
            if actions is not None:digest.update(np.asarray(actions,dtype='<f8').tobytes())
            model_key=(ref.stream_id,value.request_id);model_digest=digest.digest()
            if model_key in recorded_models:
                if recorded_models[model_key]!=model_digest:raise ValueError('model request identity reused with different recorded data')
                return
        if ref==previous:return
        if previous is not None and ref.sequence<=previous.sequence:
            if prefill or ref.sequence<=prefilled_until.get(ref.stream_id,0):return
            errors.setdefault(ref.stream_id,'source sequence regressed');return
        if (source['kind']!='image' and not(activity is not None and ref.stream_id=='control')
                and previous is not None and ref.sequence!=previous.sequence+1
                and (activity is None or written_groups.get(ref.stream_id)==group)):
            errors.setdefault(ref.stream_id,f'source sequence gap: {previous.sequence} -> {ref.sequence}')
        if source['kind']=='image':
            writer=segments.get(ref.stream_id)
            if writer is None:writer=segments[ref.stream_id]=SegmentWriter(root/'camera',ref.stream_id,
                pixel_format='rgb8-depth16le'if value.get('depth')is not None else'rgb8')
            meta=dict(value['metadata'],schema='workbench.ros-image.v1',ros_reference=ref.__dict__)
            frame=writer.add_pixels(meta,value['rgb'],value.get('depth'),value['width'],value['height'])
            append(ref,dict(frame_ref=dict(segment='camera/'+frame['segment'],frame=frame['frame']),metadata=meta))
        elif source['kind']=='model_input':
            payload=_plain(value)
            append(ref,payload)
            reconstruct_input(value,recorded_inputs if selected is None else
                SimpleNamespace(resolve_observation=lambda *args,**kwargs:selected))
        elif source['kind']=='model_result':
            directory=root/'model'/value.request_id;directory.mkdir(parents=True,exist_ok=True)
            if value.request_id in model_results:raise ValueError('duplicate model output identity')
            artifact=directory/'output.npz'
            with artifact.open('xb')as out:
                np.savez(out,action14=np.asarray(value.actions).reshape(value.rows,value.columns))
                out.flush();os.fsync(out.fileno())
            payload=_plain(value);payload.pop('actions');payload['output_ref']=dict(path=str(artifact.relative_to(root)),**_capture_file(artifact))
            model_results.add(value.request_id);append(ref,payload)
        elif source['kind']=='event':
            with artifact_lock:
                if artifact_index is None:artifact_index=(root/'artifacts.jsonl').open('x')
                payload=_archive_capture_payload(json_value(value.payload_json),root,artifact_index)
                artifact_index.flush()
            append(ref,dict(reference=ref.__dict__,event=value.event,data=payload,
                            run_id=value.run_id,epoch=value.epoch))
        else:append(ref,dict(reference=ref.__dict__,data=_plain(value)))
        written_groups[ref.stream_id]=group
        if model_key is not None:recorded_models[model_key]=model_digest
    pending=bytearray(remaining)
    try:
        health_node,health_source,health_publisher,health_declared=_recording_health_source(root,spec,writer_identity)
        state['health_source']=health_source
        emit()
        health_thread=threading.Thread(target=health_loop,name='recording-health',daemon=True);health_thread.start()
        while True:
            if health_failed.is_set():raise ValueError(errors['health'])
            ready,_,_=select.select([sys.stdin],[],[],.01)
            if ready:
                block=__import__('os').read(sys.stdin.fileno(),65536)
                if not block:
                    if terminal is None:raise ValueError('resident recorder channel closed before final barrier')
                pending.extend(block)
                if len(pending)>spec['resource_budget']['message_bytes']:raise ValueError('recording command exceeds bound')
                while b'\n'in pending:
                    line,_,pending=pending.partition(b'\n');command=json.loads(line);action=command['action']
                    if action=='bind':
                        if command['binding'].get('health_source')!=health_source:raise ValueError('original recording health source binding required')
                        if binding not in(None,command['binding']):raise ValueError('recording binding changed')
                        emit()
                        if not state['ready']:raise ValueError(state.get('error')or'capture sources/resources not ready')
                        with emit_lock:
                            binding=command['binding'];write_json(root/'binding.json',binding);state['state']='bound'
                            policy=(binding.get('execution_context')or{}).get('recording_activity')
                            if policy is not None:
                                from .recording_activity import ActivityTimeline,POLICY
                                control=sources.get('control')or{}
                                if (policy!=POLICY or binding.get('business_kind')!='inference'or control.get('kind')!='event'
                                        or control['source_instance_id']!=(binding.get('execution_owner')or{}).get('runtime_nonce')):
                                    raise ValueError('declared inference activity/control source required')
                                if any(s['source_clock_id']not in(control['source_clock_id'],'host_wall')for s in raw):
                                    raise ValueError('recording activity needs the declared control or host-wall clock')
                                activity=ActivityTimeline(binding['run_id'],control['source_clock_id'])
                                observation=binding['control_policy']['parameters']['observation']
                                activity_prefix_ns=round(max(observation['state_max_age_s'],observation['max_camera_span_s'])*1e9)
                        emit()
                    elif action=='start':
                        if binding is None:raise ValueError('recording needs a Run binding')
                        if not active:
                            with emit_lock,lock:
                                active=True;state['state']='recording'
                                for source in ([]if activity is not None else raw):
                                    for ref,value in inputs.caches[source['stream_id']].history():
                                        try:value=inputs.rgbd(ref)if source['kind']=='image'else value
                                        except LookupError:continue
                                        except ValueError as error:
                                            # This is pre-admission history, not a selected
                                            # model sample. A dependency may already have
                                            # left its bounded cache; wait for a complete
                                            # pair instead of killing the writer or replacing it.
                                            if 'referenced ROS sample was evicted' in str(error):continue
                                            raise
                                        enqueue(ref,value)
                        emit()
                    elif action=='finish':
                        terminal=command;ending_at=time.monotonic()
                        with emit_lock:state['state']='sealing'
                        write_json(root/'finish-request.json',command);emit()
                    else:raise ValueError('ROS recording accepts lifecycle only; source data must arrive through ROS')
            for _ in range(128):
                try:ref,value,group,prefill,selected=events.get_nowait()
                except queue.Empty:break
                if ref is None:
                    for source in raw:
                        stream=source['stream_id']
                        if source['kind']!='image':continue
                        collect_image(stream,wait=True)
                        if stream in segments:pending_images[stream]=encoders.submit(segments[stream].flush_segment)
                    continue
                if terminal is not None:
                    final=terminal.get('final_seq_by_stream',{}).get(ref.stream_id)
                    if final is not None and ref.sequence>final:continue
                    endpoint=terminal.get('source_endpoints',{}).get(ref.stream_id,{})
                    old=last.get(ref.stream_id)
                    if endpoint.get('source_time',0)>0 and old and old.source_time_ns/1e9>=endpoint['source_time']:continue
                if sources[ref.stream_id]['kind']=='image':
                    # One in-flight frame per camera. The existing byte-bound
                    # queue retains the backlog; a slow encoder cannot create
                    # an unbounded executor queue or serialize the other cameras.
                    collect_image(ref.stream_id,wait=True)
                    pending_images[ref.stream_id]=encoders.submit(write,ref,value,group,prefill,selected)
                else:
                    try:write(ref,value,group,prefill,selected)
                    except Exception as error:fault(ref.stream_id,error)
            for stream in list(pending_images):collect_image(stream)
            if terminal is not None:
                targets=terminal.get('final_seq_by_stream',{})
                ends=terminal.get('source_endpoints',{})
                watermarks=observed_last if activity is not None else last
                met=all(watermarks.get(k)and watermarks[k].sequence>=v or v==0 for k,v in targets.items())
                met=met and all(watermarks.get(k)and watermarks[k].source_time_ns/1e9>=v['source_time']or v['source_time']==0 for k,v in ends.items())
                if met and events.empty()and not pending_images:break
                if time.monotonic()-ending_at>=5:
                    missing=sorted({k for k,v in targets.items()if v and (k not in last or last[k].sequence<v)}|
                        {k for k,v in ends.items()if v['source_time'] and (k not in last or last[k].source_time_ns/1e9<v['source_time'])})
                    fault('final_barrier','ROS final source watermark not reached: '+(', '.join(missing)or'pending source writes'));break
        active=False;ros.close()
        for stream in list(pending_images):collect_image(stream,wait=True)
        encoders.shutdown(wait=True)
        for stream,writer in segments.items():
            try:writer.finish()
            except Exception as error:fault(stream,error)
        health_closed.set();health_changed.set()
        if health_thread is not None:
            health_thread.join(5.)
            if health_thread.is_alive():raise RuntimeError('recording health publisher did not stop')
        journal.shutdown(wait=True)
        if pending_flush is not None:
            try:flushed.update(pending_flush.result())
            except Exception as error:fault('journal',error)
        for ref in inputs.pending_rgbd|inputs.pending_metadata:
            end=terminal.get('source_endpoints',{}).get(ref.stream_id,{}).get('source_time',0.)
            relevant=activity is None or activity.sample_group(ref.source_clock_id,ref.source_time_ns)is not None
            if relevant and ref.stream_id in first and first[ref.stream_id].source_time_ns<=ref.source_time_ns<=round(end*1e9):
                fault(ref.stream_id,'original RGB sample has no matching depth or metadata at the final barrier')
        for stream,out in files.items():out.flush();__import__('os').fsync(out.fileno());out.close()
        if artifact_index is not None:artifact_index.flush();__import__('os').fsync(artifact_index.fileno());artifact_index.close()
        for stream,source in sources.items():
            if source['kind']=='image':
                end=terminal.get('source_endpoints',{}).get(stream,{}).get('source_time',0.)
                write_json(root/(stream+'-source.json'),dict(received=received[stream],committed=received[stream],
                    endpoint_reached=not end or stream in observed_last and observed_last[stream].source_time_ns/1e9>=end,
                    error=errors.get(stream),cleanup_error=None))
        finish=dict(terminal,final_seq_by_stream=dict(received),source_watermarks={k:v.__dict__ for k,v in observed_last.items()})
        if activity is not None:
            if activity.current is not None:
                end=(terminal.get('source_endpoints',{}).get('control')or{}).get('source_time')
                control=last.get('control');target=terminal.get('final_seq_by_stream',{}).get('control')
                if end is not None:activity.finish(round(end*1e9))
                elif control is not None and target is not None and control.sequence==target:activity.finish(control.source_time_ns)
                else:raise ValueError('activity final control watermark required')
            finish['recording_activity']=activity.manifest()
            state['recording_activity']=activity.summary()
        write_json(root/'finish.json',finish)
        state.update(ingress_closed=True,ready=False,source_errors=dict(errors),degraded=bool(errors))
        write_json(root/'status.json',state);emit(persist=False)
        manifest=_seal_capture(root,spec,'; '.join(errors.values())or terminal.get('control_terminal',{}).get('recording_error'))
        state.update(state=manifest['recording_outcome'],done=True,ready=False,manifest_ref='manifest.json',
            seal_revision=manifest['seal_revision'],streams=manifest['streams'],source_errors=dict(errors),degraded=bool(manifest['issues']))
        write_json(root/'status.json',state);emit(persist=False)
    finally:
        health_closed.set();health_changed.set()
        if health_thread is not None:health_thread.join(5.)
        if health_node is not None and not (health_thread is not None and health_thread.is_alive()):
            if health_source is not None:health_declared.publish(descriptor_message(health_node,dict(health_source['source'],available=False)))
            health_node.close()
        if not ros.closed:ros.close()
        encoders.shutdown(wait=True,cancel_futures=True)
        journal.shutdown(wait=True,cancel_futures=True)
        for writer in segments.values():
            if not writer.closed:writer.abort()
        for out in files.values():
            if not out.closed:out.close()
        if artifact_index is not None and not artifact_index.closed:artifact_index.close()
