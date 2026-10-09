"""Existing calibration HTTP protocol uses the shared Owner and fake reader."""
from pathlib import Path
import json
import sys
import time
import unittest
from tempfile import TemporaryDirectory
from urllib.request import Request,build_opener,ProxyHandler
from urllib.error import HTTPError

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra')]
from robot_workbench.service import protocol_fake
from robot_workbench.bridges import start_control_gateway,CameraMedia
from arx_env.joint_head_workbench_hardware import HardwareReadAdapter


class SharedBridgeTest(unittest.TestCase):
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
                device.submit(device.runtime.holder,{'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
                device.thread.join(1);device.runtime.close();device.closed.set()
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
