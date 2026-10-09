#!/usr/bin/env python3
"""Loopback-only fake device for production owner/gateway/capture integration.

No SDK, CAN, camera device, pedal or physical dynamics. Feedback is an ideal
command-following plant; RGB comes from the independently timestamped DSW RTX
renderer. Use a separate fake robot_id and calibration root in the workbench.
"""
import argparse
from collections import deque
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'infra'))
sys.path.insert(0, str(ROOT))
from examples.scripts.debug.joint_head_real_owner import Owner, SimulatedBackend, handler as owner_handler
from arx_env.joint_head_workbench_control import Bridge, handler as gateway_handler
from arx_env.joint_head_jaw_mapping import COMMAND, FEEDBACK, GAP_MM, interpolate
from arx_env.joint_head_jaw_mapping import feedback_sdk, command_gap_mm, configure_mapping
from arx_env.joint_head_workbench_geometry import Geometry


class FakeSafety:
    """Test double only: this does not validate the production pedal implementation."""
    class TakeoverState(str, Enum):
        READY='READY'; STOPPED='STOPPED_DAMPING_HOLD'; MANUAL='MANUAL_TAKEOVER'
        FAULT='FAULT_STOP_UNCONFIRMED'; RELEASED='RELEASED'

    class PedalTakeover:
        def __init__(self, backend, stop_damping_scale=1.):
            self.backend=backend;self.state=FakeSafety.TakeoverState.READY
            self.reason='fake device';self.auto_aborted=False
        def press(self):
            self.auto_aborted=True
            self.state=FakeSafety.TakeoverState.MANUAL if self.state==FakeSafety.TakeoverState.STOPPED else FakeSafety.TakeoverState.STOPPED
            self.backend.set_damping_hold(manual=self.state==FakeSafety.TakeoverState.MANUAL)
            return self.state
        def fault(self, reason):
            self.auto_aborted=True;self.reason=str(reason);self.state=FakeSafety.TakeoverState.FAULT
            self.backend.set_damping_hold(manual=False)
        def release(self):
            self.backend.close();self.state=FakeSafety.TakeoverState.RELEASED


class FakePlant(SimulatedBackend):
    # A real controller reports encoder positions; it never reports a value a float
    # epsilon below its own hard limit. Parked joints here are computed (quaternion /
    # trig round trips), so "0" arrives as -2.22e-16. The owner's motion monitor
    # validates measured qpos against the plan's SDK limits with `low <= value`, so
    # that noise latched FAULT_STOP_UNCONFIRMED ("measured qpos[8] outside SDK limit").
    # Clean the noise at the device boundary instead of loosening the safety check:
    # anything this small is indistinguishable from zero physically (1e-12 rad is
    # ~1e-7 arcsec), while a genuine out-of-limit value is orders of magnitude larger
    # and still trips the monitor.
    FLOAT_NOISE = 1e-12

    @staticmethod
    def clean(values):
        return [0. if abs(v) < FakePlant.FLOAT_NOISE else v for v in values]

    def __init__(self, state, limits):
        q=state['q'];gaps=state['jaw_gap_mm'];raw=[]
        for i,side in enumerate(('left','right')):
            raw.extend(q[i*6:i*6+6]);raw.append(feedback_sdk(side,gaps[i]))
        super().__init__(self.clean(raw))
        self.events=deque(maxlen=32);self.limits=limits
    def get_bimanual_joint_position_limits(self):
        return self.limits
    def send(self, side, q):
        self.command_guard()
        i=0 if side=='left' else 7
        self.q[i:i+6]=self.clean(q[i:i+6])
        gap=command_gap_mm(side,q[i+6])
        self.q[i+6]=feedback_sdk(side,gap)


class FakeOwner(Owner):
    def tick(self):
        alive=super().tick()
        # A completed fake trajectory needs no motor hold monitor. Leaving its
        # READY runner active made one 105 ms idle scheduler pause latch a fault.
        if (alive and self.runner and self.runner.status=='READY' and not self.sequence_remaining
                and self.capture_sample is None and self.capture_wait is None and self.arrival_pending is None):
            self.runner=None
        return alive

    def snapshot(self):
        return {**super().snapshot(),'simulation_only':True,
                'evidence_source':'fake_hardware_dsw_rtx','physical_hardware_connected':False}


def camera_metadata(header, view, serial, clock_offset_ns=0):
    m=json.loads(header)
    if (m.get('source')!='dsw_rtx_kinematic_simulation' or m.get('view')!=view
            or type(m.get('frame_sequence')) is not int or len(m.get('q_reported_rad',[]))!=12):
        raise ValueError('atomic rendered frame identity required')
    k=dict(m['camera'])
    if k.get('rectified') is not True or any(k.get('coefficients',[1])):
        raise ValueError('fake raw pinhole source must be rectified with zero distortion')
    k.update(serial=serial,rectified=False,coefficients=[0.]*5,distortion_model='distortion.inverse_brown_conrady')
    return dict(ready=True,raw=True,annotated=False,image_domain='raw',serial=serial,
                intrinsics=k,frame_number=m['frame_sequence'],
                host_receive_ns=round(m['timestamp_s']*1e9)+clock_offset_ns,
                device_timestamp_ms=m['timestamp_s']*1000,timestamp_domain='fake_render_clock',
                rendered_frame=m,clock_offset_ns=clock_offset_ns,simulation_only=True,
                source='fake_hardware_dsw_rtx',note='zero-distortion fake pinhole; not real raw sensor or exposure synchronization')


class Cameras:
    def __init__(self, config, offset):
        self.config=config;self.offset=offset;self.frames={};self.lock=threading.Lock()
        self.http=build_opener(ProxyHandler({}))
        self.live=config['live_backend'];self.token=Path(self.live['token_file']).read_text().strip()
    def request(self, path, body=None):
        return self.http.open(Request(self.live['url']+path,data=None if body is None else json.dumps(body).encode(),
            headers={'Authorization':'Bearer '+self.token,'Content-Type':'application/json'}),timeout=2)
    def poll(self, stop):
        while not stop.is_set():
            for view in ('head','left','right'):
                try:
                    with self.request('/frame.jpg?view='+view) as response:
                        raw=response.read(8*1024*1024+1);header=response.headers.get('X-Rendered-Frame')
                    if not header or len(raw)>8*1024*1024 or not raw.startswith(b'\xff\xd8') or not raw.endswith(b'\xff\xd9'):
                        raise ValueError('bounded JPEG and atomic renderer metadata required')
                    m=camera_metadata(header,view,self.config['hardware_camera_serials'][view],self.offset)
                    with self.lock:self.frames[view]=(raw,m)
                except Exception:
                    pass  # Preserve actual timestamp; stale frames fail below, never retimestamp.
            stop.wait(.01)
    def read(self, view):
        with self.lock:
            if view not in self.frames:raise ValueError('renderer camera not ready')
            raw,m=self.frames[view]
        if not 0<=(time.time_ns()-m['host_receive_ns'])/1e9<2:raise ValueError('renderer camera stale or clocks disagree')
        return raw,m


def camera_handler(cameras, view):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def do_GET(self):
            if self.path=='/stream.mjpg':
                # Mirror the real camera's MJPEG contract. Without this the workbench
                # falls back to single-frame reads only, so the streaming path it uses
                # for the live hardware view is never exercised against a fake device.
                try:
                    self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers()
                    while True:
                        try:raw,m=cameras.read(view)
                        except ValueError:
                            # Not fresh yet: hold the connection and keep trying, the way
                            # a real camera keeps streaming through a stalled frame.
                            time.sleep(1/60);continue
                        head=(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(raw)).encode()
                              +b'\r\nX-Camera-Metadata: '+json.dumps(m,separators=(',',':')).encode()+b'\r\n\r\n')
                        self.wfile.write(head+raw+b'\r\n');self.wfile.flush();time.sleep(1/30)
                except OSError:
                    pass
                return
            try:
                if self.path not in ('/status','/frame.jpg','/raw-frame.jpg'):raise ValueError('unknown camera endpoint')
                raw,m=cameras.read(view);body=json.dumps(m).encode() if self.path=='/status' else raw
                self.send_response(200);self.send_header('Content-Type','application/json' if self.path=='/status' else 'image/jpeg')
                self.send_header('Content-Length',str(len(body)));self.send_header('X-Camera-Metadata',json.dumps(m,separators=(',',':')))
                self.end_headers();self.wfile.write(body)
            except (ValueError,OSError) as error:
                body=json.dumps({'ready':False,'error':str(error)}).encode();self.send_response(503)
                self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    return Handler


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--clock-offset-ns',type=int,default=0)
    for name,port in [('owner',18301),('gateway',18302),('head',18303),('left',18304),('right',18305)]:p.add_argument('--'+name+'-port',type=int,default=port)
    a=p.parse_args();c=json.loads(a.config.read_text());root=a.output.resolve()
    if c.get('jaw_mapping_file'):configure_mapping(c['jaw_mapping_file'])
    if not c.get('robot_id','').startswith('fake-') or not Path(c['calibration_root']).resolve().is_relative_to(root.parent):
        raise ValueError('isolated fake robot and calibration root required')
    root.mkdir(exist_ok=False);log_lock=threading.Lock()
    def event(name,**details):
        with log_lock,(root/'events.jsonl').open('a') as f:f.write(json.dumps({'at_ns':time.time_ns(),'event':name,**details})+'\n')
    cameras=Cameras(c,a.clock_offset_ns)
    with cameras.request('/state') as response:state=json.load(response)
    if state['status']!='READY' or state.get('mirror'):raise ValueError('idle unowned renderer required')
    if not state.get('wrist_contract_sha256'):
        from joint_head_wrist_fixture_bringup import nominal_fixture_contract
        with cameras.request('/wrist-config',{'wrist_contract':nominal_fixture_contract(c)}) as response:
            json.load(response)
    g=Geometry(c);limits=[]
    for i in (0,6):limits.extend(g.limits[i:i+6].tolist());limits.append([0.,.082])
    plant=FakePlant(state,limits);owner=FakeOwner(plant,FakeSafety,hardware=True,event=event)
    bridge=Bridge(a.owner_port,start_config=None);token=Path(c['hardware_control']['token_file']).read_text().strip()
    stop=threading.Event();servers=[]
    for port,h in [(a.owner_port,owner_handler(owner)),(a.gateway_port,gateway_handler(bridge,token)),*[(getattr(a,side+'_port'),camera_handler(cameras,side)) for side in ('head','left','right')]]:
        server=ThreadingHTTPServer(('127.0.0.1',port),h);server.daemon_threads=True
        threading.Thread(target=server.serve_forever,daemon=True).start();servers.append(server)
    threading.Thread(target=cameras.poll,args=(stop,),daemon=True).start()
    deadline=time.monotonic()+5.0
    while time.monotonic()<deadline:
        with cameras.lock:
            ready=all(view in cameras.frames for view in ('head','left','right'))
        if ready:break
        time.sleep(.02)
    else:
        stop.set();raise RuntimeError('renderer camera first-frame readiness timeout')
    (root/'process.json').write_text(json.dumps({'pid':os.getpid(),'start_ticks':Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19], 'simulation_only':True,'argv':sys.argv},indent=2))
    print('fake hardware ready',flush=True)
    try:
        while owner.tick():stop.wait(.01)
    finally:
        stop.set()
        for server in servers:server.shutdown()

if __name__=='__main__':main()
