"""Existing calibration HTTP protocol uses the shared Owner and fake reader."""
from pathlib import Path
from copy import deepcopy
from http.client import HTTPConnection
import json
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from tempfile import TemporaryDirectory
from urllib.request import Request,build_opener,ProxyHandler
from urllib.error import HTTPError

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra')]
from robot_workbench.service import protocol_fake
from robot_workbench.bridges import start_control_gateway,CameraMedia
from arx_env.joint_head_workbench_hardware import HardwareReadAdapter


class SharedBridgeTest(unittest.TestCase):
    def test_collection_operator_preserves_legacy_16MiB_request_limit(self):
        with TemporaryDirectory() as folder:
            device=protocol_fake(Path(folder))
            from test_frozen_executor import fixture
            from frozen_executor import FrozenExecutor
            from integration import _digest
            source,plan=fixture();count=17000
            plan['ticks']=[dict(tick=i,time_s=i/30,target=deepcopy(plan['ticks'][0]['target'])) for i in range(count)]
            plan.update(native_substeps=[8]*count,duration_s=count/30,timeline_digest=_digest(plan['ticks']))
            payload=dict(action='preview_frozen',source=source,plan=plan)
            body=json.dumps(payload,allow_nan=False).encode()
            self.assertGreater(len(body),4*1024*1024);self.assertLessEqual(len(body),16*1024*1024)
            calls=[]
            def dispatch(business,command):
                self.assertEqual(business,'collection');self.assertEqual(command['action'],'preview_frozen')
                from robot_workbench.control_plan import RawPreview
                self.assertIsInstance(command,RawPreview)
                # This mocked receiver is an explicit CPU audit; the real
                # Owner decode/initialization boundary has its own pressure test.
                command=json.loads(command.raw)
                # Validate the actual legacy frozen plan without starting its follower or SDK.
                frozen=FrozenExecutor(command['plan'],command['source'],sink=lambda _:self.fail('motion started'),gate=lambda:False,stop=lambda _:None)
                self.assertEqual(len(frozen.ticks),count);calls.append(business)
                return dict(token='cpu-preview-only',timeline_digest=frozen.plan['timeline_digest'])
            tokens={'calibration':'c'*40,'collection':'d'*40}
            server,thread=start_control_gateway(device,tokens)
            try:
                with patch.object(device,'submit',side_effect=dispatch):
                    connection=HTTPConnection('127.0.0.1',server.server_port,timeout=5)
                    try:
                        connection.request('POST','/operator',body,{'Content-Type':'application/json','Authorization':'Bearer '+tokens['collection']})
                        response=connection.getresponse();reply=json.loads(response.read())
                        self.assertEqual(response.status,200,reply);self.assertEqual(reply['timeline_digest'],plan['timeline_digest'])
                    finally:connection.close()
                    # Headers alone prove an oversized body is rejected before it is read/dispatched.
                    for path,business,size in [('/operator','collection',16*1024*1024+1),('/api/trajectory-preview','calibration',len(body))]:
                        connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
                        try:
                            connection.request('POST',path,headers={'Content-Type':'application/json',
                                'Authorization':'Bearer '+tokens[business],'Content-Length':str(size)})
                            response=connection.getresponse();self.assertEqual(response.status,409)
                            self.assertIn('JSON body',json.loads(response.read())['error'])
                        finally:connection.close()
                self.assertEqual(calls,['collection']);self.assertIsNone(device.runtime)
            finally:
                device.closed.set();server.shutdown();server.server_close();thread.join(1)

    def test_collection_initial_sample_after_handoff_uses_cached_feedback_while_IO_blocks(self):
        with TemporaryDirectory() as folder:
            device=protocol_fake(Path(folder))
            tokens={'calibration':'c'*40,'collection':'d'*40}
            server,thread=start_control_gateway(device,tokens)
            url='http://127.0.0.1:'+str(server.server_port);http=build_opener(ProxyHandler({}));seen=[]
            sdk_entered,sdk_resume,record_entered,record_resume=(threading.Event() for _ in range(4))
            recorder_thread=None
            def read_owner():
                value=json.loads(http.open(url+'/state',timeout=.5).read());seen.append(value);return value
            try:
                request=Request(url+'/api/acquire',json.dumps({'confirmation':'ARMS_SUPPORTED_CONNECT'}).encode(),
                    headers={'Authorization':'Bearer '+tokens['calibration'],'Content-Type':'application/json'})
                http.open(request,timeout=1).read()
                from real_observation import read_initial_sample
                from arx_env.joint_head_jaw_mapping import feedback_gaps,mapping_identity
                deadline=time.monotonic()+2
                while time.monotonic()<deadline and not read_owner().get('stationary'):time.sleep(.01)
                with self.assertRaisesRegex(ValueError,'stationary'):
                    read_initial_sample(read_owner,environment='fake')
                while True:
                    try:device.runtime.handoff(device.credentials['calibration'],'collection');break
                    except ValueError:
                        if time.monotonic()>=deadline:raise
                        time.sleep(.005)
                owner=device.runtime.owner
                with patch.object(owner,'snapshot',side_effect=AssertionError('full Owner snapshot is forbidden')):
                    physical,evidence=read_initial_sample(read_owner,environment='fake')
                    raw=seen[-1]['calibration_feedback']
                    self.assertEqual(physical['left_arm_joints'],raw['qpos'][:6])
                    self.assertEqual(physical['right_arm_joints'],raw['qpos'][7:13])
                    self.assertEqual([physical['left_gripper'],physical['right_gripper']],feedback_gaps(raw['qpos']))
                    for key in ('source_clock_id','source_time_ns','physics_step','sample_wall_ns','sample_time_s'):
                        self.assertEqual(evidence[key],raw[key])
                    self.assertEqual(evidence['runtime_nonce'],device.runtime.nonce)
                    self.assertEqual(seen[-1]['collection_jaw_mapping'],mapping_identity()['version'])
                    self.assertFalse(seen[-1]['position_hold_active']);self.assertIsNone(seen[-1]['maximum_control_gap_s'])
                    self.assertIsNone(owner.runner);self.assertIsNone(device.runtime.execution_calibration)
                    backend=owner.backend._backend;original=backend.read_calibration_state
                    def stuck_read():
                        sdk_entered.set();sdk_resume.wait(3);return original()
                    def stuck_record_snapshot():
                        record_entered.set();record_resume.wait(3);return {}
                    recorder=SimpleNamespace(ready=threading.Event(),done=threading.Event(),error=None,
                        root=Path(folder)/'recording',manifest={'files':{},'capture_status':'running'},controller_result=None,
                        snapshot=stuck_record_snapshot)
                    recorder_thread=threading.Thread(target=recorder.snapshot);recorder.thread=recorder_thread
                    owner.recording=recorder;recorder_thread.start();self.assertTrue(record_entered.wait(.5))
                    with patch.object(backend,'read_calibration_state',stuck_read):
                        self.assertTrue(sdk_entered.wait(.5))
                        cached=deepcopy(owner.cache['calibration_feedback']);started=time.monotonic();blocked=read_owner()
                        self.assertLess(time.monotonic()-started,.2,'status must not wait for SDK or recorder locks')
                        self.assertEqual(blocked['calibration_feedback'],cached)
                        self.assertEqual(blocked['physical_feedback']['left_arm_joints'],cached['qpos'][:6])
                        self.assertTrue(blocked['sdk_calls_in_flight']);self.assertFalse(blocked['recording']['done'])
                        time.sleep(.12)
                        with self.assertRaisesRegex(ValueError,'fresh healthy'):
                            read_initial_sample(read_owner,environment='fake')
                        self.assertEqual(seen[-1]['calibration_feedback'],cached,'reading status must not refresh a frozen source clock')
                        sdk_resume.set()
            finally:
                sdk_resume.set();record_resume.set()
                if recorder_thread:recorder_thread.join(1)
                if device.runtime:
                    device.runtime.owner.recording=None
                    runtime=device.runtime
                    runtime.owner.submit({'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
                    device.thread.join(1);runtime.close()
                device.closed.set();server.shutdown();server.server_close();thread.join(1)

    def test_old_calibration_client_reads_fake_and_cannot_cross_business(self):
        with TemporaryDirectory() as folder:
            device=protocol_fake(Path(folder));device.connect({})
            tokens={'calibration':'c'*40,'collection':'d'*40}
            server,thread=start_control_gateway(device,tokens)
            url='http://127.0.0.1:'+str(server.server_port)
            http=build_opener(ProxyHandler({}))
            def get(path):
                return json.loads(http.open(Request(url+path,headers={'Authorization':'Bearer '+tokens['calibration']})).read())
            def post(path,body,business='calibration'):
                request=Request(url+path,json.dumps(body).encode(),headers={'Authorization':'Bearer '+tokens[business]})
                return json.loads(http.open(request).read())
            try:
                deadline=time.monotonic()+2
                while time.monotonic()<deadline and not device.snapshot('calibration').get('stationary'):time.sleep(.01)
                state=get('/api/state')
                self.assertTrue(state['motion_enabled'])
                self.assertEqual(state['owner']['mode'],'simulation')
                reader=HardwareReadAdapter(server.server_port,environment='fake')
                reader.read_state();time.sleep(.03)
                self.assertTrue(reader.read_state()['feedback']['valid'])
                physical=HardwareReadAdapter(server.server_port)
                self.assertFalse(physical.read_state()['feedback']['valid'])
                review=post('/api/trajectory-preview',dict(waypoints=[dict(q12=[.01]+[0.]*11,
                    speeds12=[.2]*12,jaw_gap_mm=[80.,80.],jaw_speed_mm_s=10.)]))
                self.assertTrue(review['token'])
                with self.assertRaises(HTTPError) as denied:
                    post('/operator',dict(action='confirm',token=review['token'],
                        confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True),'collection')
                self.assertEqual(denied.exception.code,403)
                device.runtime.handoff(device.credentials['calibration'],'collection')
                with self.assertRaises(HTTPError):
                    post('/api/motion-confirm',dict(token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                self.assertFalse(get('/api/state')['motion_enabled'])
            finally:
                runtime=device.runtime
                runtime.owner.submit({'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
                device.thread.join(1);runtime.close();device.closed.set()
                server.shutdown();server.server_close();thread.join(1)

    def test_camera_fanout_preserves_source_time_and_rejects_header_injection(self):
        metadata={'frame_number':1,'host_receive_ns':time.time_ns()}
        media=CameraMedia('head','head',lambda:(b'jpeg-fixture',dict(metadata)),environment='fake')
        media.closed.set();media.thread.join(1)
        media.refresh();before=media.frame()
        self.assertEqual(media.frame(),before)
        metadata['host_receive_ns']+=100
        with self.assertRaisesRegex(ValueError,'same frame'):media.refresh()
        metadata['frame_number']='bad\r\nInjected: yes'
        with self.assertRaises(ValueError):media.refresh()


if __name__=='__main__':unittest.main()
