"""Read-only subscriber for existing Prometheus RGB publishers; never opens a camera.

One atomic latest-frame file per camera lives in task-owned /dev/shm. Source
timestamps survive IPC; receiving an old frame cannot make it fresh again.
"""
import json
import os
from pathlib import Path
import struct
import time
import queue
import threading

import numpy as np

CAMERAS=('base_0','left_wrist_0','right_wrist_0')


def rgb_bytes(message):
    if (message.width!=640 or message.height!=480 or message.encoding not in ('bgr8','rgb8')
            or message.step<1920 or message.step>4096 or len(message.data)!=message.step*480):
        raise ValueError('raw 640x480 RGB/BGR camera frame required')
    value=np.frombuffer(message.data,np.uint8).reshape(480,message.step)[:,:1920].reshape(480,640,3)
    return (value[:,:,::-1]if message.encoding=='bgr8'else value).tobytes()


def write_frame(path,metadata,rgb):
    header=json.dumps(metadata,allow_nan=False,separators=(',',':')).encode()
    if len(header)>8192 or len(rgb)!=640*480*3:raise ValueError('bounded raw frame required')
    path=Path(path);tmp=path.with_suffix('.tmp')
    with tmp.open('wb')as f:f.write(struct.pack('!I',len(header))+header+rgb)
    tmp.replace(path)


class RGBClient:
    def __init__(self,directory,owner_path):
        self.directory,self.owner_path=Path(directory),Path(owner_path);self.descriptor={}

    def recording_service(self):
        from .rgbd_service import CommonCameraService
        return CommonCameraService.connect(self.directory/'recording-camera-service.json')

    @property
    def channels(self):return self.recording_service().channels

    def capture_sources(self):return self.recording_service().capture_sources()

    def latest_rgb(self,camera):
        from .camera_owner import read_owner
        from .control_plan import _read_file
        if camera not in CAMERAS:raise ValueError('known camera required')
        owner=read_owner(self.owner_path)
        self.descriptor={'instance_id':owner['instance']}
        raw=_read_file(str(self.directory/(camera+'.rgb')),1024*1024)
        if len(raw)<4:raise ValueError('truncated camera header')
        n=struct.unpack('!I',raw[:4])[0]
        if not 0<n<=8192 or len(raw)!=4+n+640*480*3:raise ValueError('invalid raw camera frame size')
        meta=json.loads(raw[4:4+n])
        if (meta.get('capture_instance_id')!=owner['instance']or meta.get('serial')!=owner['profile']['cameras'][camera]
                or meta.get('camera')!=camera or meta.get('pixel_format')!='rgb8'
                or type(meta.get('host_receive_ns'))is not int or not 0<meta['host_receive_ns']<=time.time_ns()):
            raise ValueError('camera identity changed or source frame stale')
        return dict(metadata=meta,rgb=raw[4+n:],width=640,height=480)


class RGBRecordingBridge:
    """Bounded subscriber handoff; latest-input writes never wait for recorder IPC."""
    def __init__(self,service):
        self.service=service;self.frames=queue.Queue(maxsize=3);self.closed=threading.Event()
        self.errors={};self.lock=threading.Lock()
        self.thread=threading.Thread(target=self._run,daemon=True,name='rgb-recording-source');self.thread.start()

    def submit(self,camera,metadata,rgb):
        if (camera not in CAMERAS or not isinstance(metadata,dict) or type(metadata.get('host_receive_ns'))is not int
                or not isinstance(rgb,bytes)or len(rgb)!=640*480*3):raise ValueError('bounded original RGB frame required')
        if self.closed.is_set():return False
        try:self.frames.put_nowait((camera,dict(metadata),rgb));return True
        except queue.Full:
            with self.lock:self.errors[camera]=(metadata['host_receive_ns'],'bounded RGB source queue overflow')
            return False

    def _run(self):
        while not self.closed.is_set()or not self.frames.empty():
            try:camera,meta,rgb=self.frames.get(timeout=.1)
            except queue.Empty:continue
            try:
                channel=self.service.channels[camera];lease=channel.current_lease()
                with self.lock:failure=self.errors.pop(camera,None)
                if lease and failure and lease.get('admission_start_wall_ns',lease['start_wall_ns'])<=failure[0]:
                    state=channel.snapshot(lease['run_id']);endpoint=state.get('admission_endpoint_ns')
                    if endpoint is None or failure[0]<=endpoint:channel.fail(failure[1],sampled_token=lease['run_id'])
                self.service.publish(camera,meta,rgb,None,width=640,height=480,sampled_token=lease['run_id']if lease else None)
            except Exception as error:
                with self.lock:self.errors[camera]=(meta['host_receive_ns'],str(error))
            finally:self.frames.task_done()

    def close(self):
        self.closed.set();self.thread.join(6)
        if self.thread.is_alive():raise TimeoutError('RGB recording source IPC has not returned')


def main():
    import argparse
    from .camera_owner import read_owner
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();source=read_owner(args.owner)
    if source['state']!='running' or source['profile']['ros_domain']!=os.environ.get('ROS_DOMAIN_ID','0'):
        raise ValueError('existing running camera publisher on the same ROS domain required')
    args.output.mkdir(mode=0o700,parents=True,exist_ok=True)
    import rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
    from .rgbd_service import CommonCameraService
    intrinsics={camera:dict(serial=source['profile']['cameras'][camera],width=640,height=480)for camera in CAMERAS}
    rclpy.init();node=rclpy.create_node('workbench_mixed366_rgb_subscriber')
    service=bridge=None
    try:
        service=CommonCameraService.start(intrinsics,args.output/('recording-'+str(time.time_ns())),'physical',
            pixel_formats=dict.fromkeys(CAMERAS,'rgb8'))
        descriptor=args.output/'recording-camera-service.json';temporary=descriptor.with_suffix('.tmp')
        with os.fdopen(os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600),'w')as stream:
            json.dump(service.descriptor,stream)
        temporary.replace(descriptor);bridge=RGBRecordingBridge(service)
        # Keep the original latest-input subscriber semantics. Recorder queue
        # gaps are explicit and do not claim unobservable upstream coverage.
        qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        def receive(camera,message):
            current=read_owner(args.owner)
            if current['instance']!=source['instance']:raise ValueError('publisher restarted; explicitly restart subscriber')
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            if not 0<stamp<=time.time_ns():
                with bridge.lock:bridge.errors[camera]=(time.time_ns(),'invalid original publisher timestamp')
                return
            meta=dict(camera=camera,serial=source['profile']['cameras'][camera],capture_instance_id=source['instance'],
                host_receive_ns=stamp,subscriber_receive_ns=time.time_ns(),frame_number=stamp,environment='physical',
                clock_domain='host_wall',pixel_format='rgb8',timestamp_basis='publisher_host_after_capture',
                ready=True,raw=True,annotated=False,data_valid=True,capture_fps=30,camera_name=camera,
                intrinsics=intrinsics[camera],pair_id=str(stamp))
            rgb=rgb_bytes(message)
            write_frame(args.output/(camera+'.rgb'),meta,rgb)
            bridge.submit(camera,meta,rgb)
        subscriptions=[node.create_subscription(Image,source['profile']['topic_prefix']+'/'+camera+'_rgb/image_raw',
            lambda message,camera=camera:receive(camera,message),qos)for camera in CAMERAS]
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        try:
            if bridge is not None:bridge.close()
        finally:
            if service is not None:service.close()


if __name__=='__main__':main()
