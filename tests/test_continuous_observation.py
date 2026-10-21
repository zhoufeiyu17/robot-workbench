"""CPU-only: continuous detection, reset boundaries, queued sampling and stream ownership."""
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path[:0] = [str(Path(__file__).resolve().parents[1]/p) for p in ('rule_based_workbench','detector_adapter')]
from robot_workbench.observation_window import ContinuousObservationWindow
from test_detection_bridge import fixture


def row(index, x=1.):
    observation=deepcopy(fixture()[1][0])
    observation.update(source_stamp_ns=1_000_000_000+index*375_000_000, source_sequence_id=index,
                       capture_sequence_id=index)
    observation['tracks'][1]['world']['xyz_m'][0]=x
    return observation


class WindowTest(unittest.TestCase):
    def test_exact_three_seconds_and_nine_frames(self):
        window=ContinuousObservationWindow()
        for i in range(8):self.assertFalse(window.add(row(i))['ready'])
        self.assertTrue(window.add(row(8))['ready'])
        self.assertEqual(window.status()['span_s'],3.)

    def test_failure_boundary_and_exact_six_mm_and_cumulative_drift(self):
        for anomaly in ('missing','six_mm','gap','identity'):
            with self.subTest(anomaly=anomaly):
                window=ContinuousObservationWindow()
                for i in range(3):window.add(row(i))
                current=row(3,1.006 if anomaly=='six_mm' else 1.)
                if anomaly=='missing':current['pair_valid']=False
                if anomaly=='gap':current['source_stamp_ns']+=1_000_000_000
                if anomaly=='identity':current['tracks'][0]['track_id']='new'
                self.assertEqual(window.add(current)['frame_count'],0)
                # Boundary is excluded; the next valid observation starts at zero.
                for i in range(4,13):
                    next_row=row(i)
                    if anomaly=='gap':next_row['source_stamp_ns']+=1_000_000_000
                    if anomaly=='missing':next_row['tracks'][0]['track_id']='new_after_missing'
                    state=window.add(next_row)
                    self.assertEqual(state['ready'],i==12)
                self.assertEqual(len(window.events),1)
        window=ContinuousObservationWindow()
        for i in range(7):state=window.add(row(i,1.+i*.001))
        self.assertFalse(state['ready']);self.assertEqual(state['frame_count'],0)
        self.assertEqual(window.events[-1]['reason'],'cup_position_span')

    def test_fatal_errors_are_not_retries(self):
        for change in ({'tracking_session_id':'foreign'}, {'runtime_config_id':'other'},
                       {'provider_status':'FAILED'}, {'source_stamp_ns':1}, {'pair_valid':'yes'}):
            window=ContinuousObservationWindow();window.add(row(0))
            with self.assertRaises(ValueError):window.add(dict(row(1),**change))

    def test_sampling_continues_during_first_detector_registration(self):
        from real_observation import capture
        from test_real_observation import fixture as rgbd
        from test_harness import PROFILE
        from integration import _digest
        from real_pipeline import validate_capture
        profile=deepcopy(PROFILE)
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
            camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
        clock=[0.];count=[0];read_while_detecting=[0];started=threading.Event();release=threading.Event()
        def read():
            index=count[0];count[0]+=1
            if started.is_set() and not release.is_set():read_while_detecting[0]+=1
            if count[0]>=340:release.set()
            return rgbd(index,lambda meta:meta.update(host_receive_ns=1_000_000_000+index*375_000_000))
        def sleep(seconds):clock[0]+=seconds;time.sleep(.002)
        calls=[]
        def detect(directory,frame,remaining):
            if not calls:started.set();self.assertTrue(release.wait(2))
            calls.append(frame['capture_sequence_id'])
            result=row(frame['capture_sequence_id'])
            result['source_stamp_ns']=frame['source_stamp_ns']
            return result
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'capture'
            sealed=capture(read,profile,binding,root,detect_frame=detect,
                timeout_s=300.,monotonic=lambda:clock[0],sleep=sleep)
            manifest=validate_capture(root,sealed)
            self.assertEqual(len(manifest['frames']),9)
            self.assertEqual(calls,list(range(9)))
            self.assertGreater(read_while_detecting[0],3)
            self.assertGreater(len(list(root.glob('*.npz'))),9)
            self.assertGreater(clock[0],120.)
            self.assertEqual(manifest['frames'][-1]['source_stamp_ns']-manifest['frames'][0]['source_stamp_ns'],3_000_000_000)

    def test_physical_deadline_reports_counts_and_resets_without_process_argv(self):
        from concurrent.futures import Future
        from real_observation import capture
        from test_real_observation import fixture as rgbd
        from test_harness import PROFILE
        from integration import _digest
        class Immediate:
            def __init__(self,**kwargs):pass
            def submit(self,callback,*args):
                future=Future()
                try:future.set_result(callback(*args))
                except Exception as error:future.set_exception(error)
                return future
            def shutdown(self,**kwargs):pass
        profile=deepcopy(PROFILE)
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
            camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
        clock=[0.];index=[0];calls=[]
        def read():
            i=index[0];index[0]+=1
            return rgbd(i,lambda meta:meta.update(host_receive_ns=1_000_000_000+i*375_000_000))
        def detect(directory,frame,remaining):
            i=frame['capture_sequence_id'];calls.append(i)
            if i==3:
                clock[0]=20.
                raise subprocess.TimeoutExpired(['ssh','owner_token=private-token','base64-'+'A'*10000],remaining)
            observation=row(i)
            if i==1:observation['pair_valid']=False
            return observation
        with tempfile.TemporaryDirectory() as folder,patch('real_observation.ThreadPoolExecutor',Immediate):
            output=Path(folder)/'capture'
            with self.assertRaises(TimeoutError) as raised:
                capture(read,profile,binding,output,detect_frame=detect,timeout_s=120.,
                    wall_deadline_ns=21_000_000_000,wall_ns=lambda:1_000_000_000+int(clock[0]*1e9),
                    monotonic=lambda:clock[0],sleep=lambda seconds:clock.__setitem__(0,clock[0]+seconds))
            message=str(raised.exception)
            for expected in ('累计时间预算已用尽','已采 4 帧','已判定 3 帧','0.000/3.000','重新计时 1 次','检出失败'):
                self.assertIn(expected,message)
            self.assertLess(len(message),200)
            self.assertNotIn('private-token',message);self.assertNotIn('base64',message)
            diagnostic=json.loads((output/'capture-diagnostic.json').read_text())
            self.assertEqual(diagnostic['elapsed_wall_s'],20.)
            self.assertEqual(diagnostic['processed_frame_count'],3)
            self.assertEqual(diagnostic['observation_window']['frame_count'],1)
            self.assertTrue(diagnostic['reobserve_events'][0]['boundary_frame_excluded'])
            self.assertEqual(calls,[0,1,2,3])
            self.assertFalse((output/'rgbd-manifest.json').exists())

    def test_stream_rpc_errors_are_short_and_retain_private_cause_without_retry(self):
        from real_acquisition import StreamingDetection
        pipeline=dict(detection_script='/approved/detect.py',detector_service=dict(socket='/approved/socket',
            owner_token='private-token',identity={'adapter':'test'}))
        detector=StreamingDetection(pipeline,{},Path('/unused'),transfer=lambda **kw:None,cancel=None,progress=None)
        argv=['ssh','owner_token=private-token','base64-'+'A'*10000]
        for underlying in (subprocess.TimeoutExpired(argv,.07),subprocess.CalledProcessError(255,argv,stderr=b'private-token')):
            with self.subTest(type=type(underlying).__name__),patch('run_pipeline.transport',return_value=argv),\
                    patch('cancellable_process.run',side_effect=underlying) as command:
                with self.assertRaises((TimeoutError,RuntimeError)) as raised:
                    detector.rpc({'operation':'fixture'},.07)
                self.assertIs(raised.exception.__cause__,underlying)
                self.assertIn('检测服务调用',str(raised.exception))
                self.assertLess(len(str(raised.exception)),150)
                self.assertNotIn('private-token',str(raised.exception));self.assertNotIn('base64',str(raised.exception))
                self.assertEqual(command.call_count,1)

    def test_fatal_detector_failure_never_seals_and_keeps_diagnostics(self):
        from real_observation import capture
        from test_real_observation import fixture as rgbd
        from test_harness import PROFILE
        from integration import _digest
        profile=deepcopy(PROFILE)
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
            camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
        for failure in (RuntimeError('provider process failed'), InterruptedError('cancelled')):
            with self.subTest(failure=type(failure).__name__), tempfile.TemporaryDirectory() as folder:
                clock=[0.];index=[0]
                def read():
                    i=index[0];index[0]+=1
                    return rgbd(i,lambda meta:meta.update(host_receive_ns=1_000_000_000+i*375_000_000))
                def sleep(seconds):clock[0]+=seconds;time.sleep(.002)
                def detect(*args):raise failure
                output=Path(folder)/'capture'
                with self.assertRaises(type(failure)):
                    capture(read,profile,binding,output,detect_frame=detect,monotonic=lambda:clock[0],sleep=sleep)
                self.assertFalse((output/'rgbd-manifest.json').exists())
                diagnostic=json.loads((output/'capture-diagnostic.json').read_text())
                self.assertEqual(diagnostic['error_type'],type(failure).__name__)
                self.assertFalse(diagnostic['manifest_sealed'])

    def test_acquisition_seals_live_detection_and_bridge_does_not_redetect(self):
        from integration import _digest
        from real_acquisition import prepare_inputs, StreamingDetection
        from real_observation import capture as real_capture
        from test_real_observation import fixture as rgbd
        from real_pipeline import import_capture, bind_detection, preparation_stages
        from source_from_detection import convert
        from run_pipeline import PATH_KEYS
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder).resolve();remote=root/'inputs';remote.mkdir()
            profile=fixture()[2]
            binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
                camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
            for name,value in (('profile',profile),('binding',binding)):
                (root/(name+'.json')).write_text(json.dumps(value))
            contract=dict(filesystem=dict(writable_roots=[str(remote)],readonly_roots=[str(remote),'/approved']),
                resources=dict(max_wall_time_minutes=20),provenance=dict(input_sha256={},config_sha256={}))
            (root/'contract.json').write_text(json.dumps(contract))
            pipeline={key:'/approved/'+key for key in PATH_KEYS}
            pipeline.update(contract_path=str(root/'contract.json'),remote_run_root='/mnt/data/feiyu.zhou/runs/fixture/jobs/run',
                opening_mm=83,finger_envelope_sha256='a'*64,render_replay=False,
                detector_service=dict(allowed_output_root=str(root/'jobs'),socket='/approved/socket',
                    owner_token='fixture-owner',identity={'adapter_sha256':'sha256:'+'a'*64}))
            (root/'pipeline.json').write_text(json.dumps(pipeline))
            config=dict(pipeline_config=str(root/'pipeline.json'),acquisition=dict(profile_path=str(root/'profile.json'),
                acquisition_binding_path=str(root/'binding.json'),transport=dict(remote_root=str(remote),endpoint=None)))
            observed=[];ended=[];clock=[0.];index=[0]
            def read():
                i=index[0];index[0]+=1
                return rgbd(i,lambda meta:meta.update(host_receive_ns=1_000_000_000+i*375_000_000))
            def sleep(seconds):clock[0]+=seconds;time.sleep(.005)
            def capture(*args,**kwargs):return real_capture(*args,**kwargs,monotonic=lambda:clock[0],sleep=sleep)
            def rpc(detector,payload,timeout):
                if payload.get('stream_end'):
                    ended.append(detector.stream_id)
                    return dict(stream_id=detector.stream_id,cleanup_complete=True)
                value=json.loads(Path(payload['manifest']).read_text());frame=value['frames'][0]
                observation=row(frame['capture_sequence_id'])
                if frame['capture_sequence_id']==1:observation['pair_valid']=False
                observed.append(observation)
                summary=dict(fixture()[0],status='DETECTION_FRAME_READY',tracking_session_id='fixture',
                    inputs=[dict(frame,sha256='sha256:'+frame['sha256'])],model_reused=True,
                    service=dict(pid=7,prepare_count=1),manifest_sha256='sha256:'+str(len(observed)))
                return dict(stream_id=detector.stream_id,summary=summary,observation=observation)
            with patch('real_observation.capture',capture),patch.object(StreamingDetection,'rpc',rpc):
                prepared=prepare_inputs(config,root/'acquire',read_snapshot=read)
            self.assertTrue(prepared['online_detection']);self.assertEqual(len(ended),1)
            frozen=json.loads(Path(prepared['pipeline_config']).read_text())
            frozen_contract=json.loads(Path(frozen['contract_path']).read_text())
            stages=preparation_stages(frozen,frozen_contract,prepared['capture_dir'],prepared['physical_binding'],scene_only=True)
            self.assertNotIn('detect',[s['name'] for s in stages])
            target=root/'prepared';target.mkdir()
            import_capture(prepared['capture_dir'],prepared['physical_binding'],target/'capture')
            bind_detection(target)
            result=convert(target/'capture/profile.json',target/'detection',target/'source','fixture',
                json.loads((target/'physical-detection-binding.json').read_text()))
            source=result['scene']['source']
            self.assertEqual(source['window_start_ns'],1_750_000_000)
            self.assertEqual(source['window_end_ns'],4_750_000_000)
            self.assertEqual(source['sample_count'],9)
            self.assertEqual(source['reobserve_count'],1)
            self.assertEqual(len(observed),11)
            # Mutating sealed detector output is rejected before any downstream use.
            (Path(prepared['capture_dir'])/'detection/summary.json').write_text('{}')
            (root/'tampered').mkdir()
            with self.assertRaisesRegex(ValueError,'detection evidence'):
                import_capture(prepared['capture_dir'],prepared['physical_binding'],root/'tampered/capture')

    def test_private_stream_resets_tracking_only_on_begin_end(self):
        import isaac_detection_service as service
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder).resolve();(root/'code').mkdir();(root/'inputs').mkdir();(root/'outputs').mkdir()
            adapter_path=root/'code/adapter.py';adapter_path.write_text('fixture')
            config=root/'node.json';config.write_text('{}')
            frame=root/'inputs/frame.npz';frame.write_bytes(b'fixture')
            manifest=root/'inputs/manifest.json';manifest.write_text('{}')
            reset=[0]
            class Runtime:
                @property
                def tracking_session_id(self):return str(reset[0])
                def new_session(self,timeout):reset[0]+=1;return self
                def close(self,timeout):return True
            runtime=Runtime()
            context=dict(runtime=runtime,catalog=NS(runtime_config=NS(sensor_profile=NS(optical_frame='camera'))))
            runs=[]
            def run(args,prepared):
                runs.append((args.streaming,runtime.tracking_session_id))
                args.run_dir.mkdir()
                observation=row(len(runs));observation['tracking_session_id']=runtime.tracking_session_id
                (args.run_dir/'observations.jsonl').write_text(json.dumps(observation)+'\n')
                return dict(tracking_session_id=runtime.tracking_session_id,inputs=[{}])
            adapter=NS(__file__=str(adapter_path), initialize_detection=lambda *a:(context,0),
                load_manifest=lambda *a,**k:dict(frame_id='camera',frames=[dict(path='frame.npz')]),
                read_frame=lambda *a:None,run=run,write_json=lambda p,v:p.write_text(json.dumps(v)))
            identity=dict(adapter_sha256=service.digest(adapter_path),service_sha256=service.digest(service.__file__),node_config_sha256=service.digest(config))
            requests=[]
            a='a'*32;b='b'*32
            for i,(stream,begin,end) in enumerate(((a,True,False),(a,False,False),(b,False,False),(a,False,True),(b,True,False))):
                requests.append(dict(owner_token='owner',expected_identity=identity,stream_id=stream,
                    stream_begin=begin,stream_end=end,result_timeout_s=1,manifest=str(manifest),run_dir=str(root/'outputs'/str(i))))
            responses=[]
            class Connection:
                def __init__(self,payload):self.payload=payload
                def __enter__(self):return self
                def __exit__(self,*a):pass
                def settimeout(self,*a):pass
                def recv(self,*a):return json.dumps(self.payload).encode()+b'\n'
                def sendall(self,data):responses.append(json.loads(data))
            class Listener:
                def bind(self,address):Path(address).touch()
                def listen(self,*a):pass
                def close(self):pass
                def accept(self):
                    if not requests:raise KeyboardInterrupt()
                    return Connection(requests.pop(0)),None
            args=NS(service_root=root/'service',socket=root/'service/socket',owner_token='owner',node_config=config,
                allowed_read_root=[root/'inputs'],allowed_output_root=root/'outputs')
            original_read = Path.read_text
            def read_text(path,*args,**kwargs):
                if str(path)=='/proc/self/stat':return '1 (fixture) '+' '.join(['S']+['0']*18+['123'])
                if str(path)=='/proc/sys/kernel/random/boot_id':return '00000000-0000-0000-0000-000000000001'
                return original_read(path,*args,**kwargs)
            with patch.object(Path,'read_text',read_text),patch('sys.platform','linux'),patch.dict(os.environ,{'CUDA_VISIBLE_DEVICES':'0'}),\
                    patch.object(service.socket,'socket',return_value=Listener()),patch.object(service.signal,'signal'),\
                    patch.object(service,'model_count',return_value=1),patch('builtins.print'):
                with self.assertRaises(KeyboardInterrupt):service.serve(args,adapter)
            self.assertEqual([r.get('exit_code') for r in responses],[0,0,2,0,0])
            self.assertEqual([r[1] for r in runs],['2','2','4'])
            self.assertEqual(reset[0],4)
            self.assertTrue(responses[3]['cleanup_complete'])


if __name__=='__main__':unittest.main()
