"""Keep the original RealSense publisher alive independently of collection clients."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

PROFILE_KEYS = ('topic_prefix', 'width', 'height', 'ir_width', 'ir_height', 'fps', 'enable_ir', 'qos_depth')


def profile(args):
    return dict(cameras=json.loads(args.cameras), ros_domain=os.environ.get('ROS_DOMAIN_ID', '0'),
                **{key: getattr(args, key) for key in PROFILE_KEYS})


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp.' + str(os.getpid()))
    temporary.write_text(json.dumps(value) + '\n')
    temporary.replace(path)


def read_owner(path, expected=None, *, allow_stopped=False):
    value = json.loads(Path(path).read_text())
    age = time.monotonic() - value['heartbeat']
    if not 0 <= age <= 4 and not (allow_stopped and value['state'] in ('stopped', 'error')):
        raise ValueError('独立相机服务心跳中断')
    if expected is not None and value['profile'] != expected:
        differences = ', '.join(key for key in expected if value['profile'].get(key) != expected[key])
        raise ValueError('独立相机配置与数采不一致（' + differences + '）；拒绝重复打开硬件，不会自动降低录制规格')
    if value['state'] not in ('starting', 'running') and not allow_stopped:
        raise ValueError('独立相机服务已停止：' + value.get('error', ''))
    return value


def reuse_camera_owner(args, path, *, startup_timeout=20):
    """A collection-owned client exits without terminating the shared hardware owner."""
    expected, instance = profile(args), None
    ready = Path(args.ready_file) if args.ready_file else None
    deadline = time.monotonic() + startup_timeout
    try:
        while True:
            value = read_owner(path, expected)
            if instance is None:
                instance = value['instance']
            if value['instance'] != instance:
                raise ValueError('独立相机服务已重启，需要重新启动本批采集')
            if value['state'] == 'running':
                if ready is not None and not ready.exists():
                    atomic(ready, dict(status='ready', subsystem='cameras', devices=sorted(expected['cameras'])))
            elif time.monotonic() >= deadline:
                raise ValueError('等待独立相机首帧超时')
            time.sleep(.25)
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, KeyError) as error:
        print('Shared camera error:', error, file=sys.stderr, flush=True)
        return 1
    finally:
        if ready is not None:
            ready.unlink(missing_ok=True)


def run_owner(state, args):
    """Only the original publisher opens SDKs; systemd owns the entire process group."""
    state = Path(state)
    import fcntl
    state.parent.mkdir(parents=True, exist_ok=True)
    lock = state.with_suffix('.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    ready = state.with_suffix('.ready.json')
    ready.unlink(missing_ok=True)
    value = dict(instance=uuid.uuid4().hex, pid=os.getpid(), profile=profile(args), state='starting', error='')
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    command = [sys.executable, '-m', 'prometheus.sensors.realsense_publisher', '--cameras', args.cameras]
    for key in PROFILE_KEYS:
        if key != 'enable_ir':
            command.extend(['--' + key, str(getattr(args, key))])
    command.extend(['--enable_ir' if args.enable_ir else '--no-enable_ir', '--ready_file', str(ready)])
    child = None;ros=None;source_publishers={}
    if getattr(args,'ros_namespace',None):
        from robot_workbench.ros_data import RosNode
        if not getattr(args,'device_id',None):raise ValueError('ROS camera source requires explicit device_id')
        ros=RosNode('camera_sources',namespace=args.ros_namespace,
            domain_id=int(os.environ.get('ROS_DOMAIN_ID','0')),localhost_only=False)
    def publish_sources(available):
        if ros is None:return
        from robot_workbench.ros_data import descriptor_message
        for camera,serial in value['profile']['cameras'].items():
            if camera not in source_publishers:
                source_publishers[camera]=ros.publisher('sources/'+camera,ros.types.SourceDescriptor,retained=True,depth=1)
            source_publishers[camera].publish(descriptor_message(ros,dict(device_id=args.device_id,stream_id=camera,
                source_instance_id=value['instance'],source_clock_id='host_wall',
                topic=args.topic_prefix+'/'+camera+'_rgb/image_raw',message_type='sensor_msgs/msg/Image',
                kind='image',units='rgb8',coordinate_frame=camera,nominal_hz=float(args.fps),available=available,
                metadata=dict(serial=serial,width=args.width,height=args.height,environment='physical',
                    timestamp_basis='publisher_host_after_capture'))))
    try:
        atomic(state, dict(value, heartbeat=time.monotonic()))
        child = subprocess.Popen(command, env=dict(os.environ, ROBOT_WORKBENCH_CAMERA_OWNER='1'), start_new_session=True)
        value['publisher_pid'] = child.pid
        while not stopped and child.poll() is None:
            value['state'] = 'running' if ready.is_file() else 'starting'
            atomic(state, dict(value, heartbeat=time.monotonic()))
            publish_sources(value['state']=='running')
            time.sleep(.5)
        if not stopped:
            value.update(state='error', error='原相机发布进程退出，代码 ' + str(child.returncode))
    finally:
        if child is not None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        if value['state'] != 'error':
            value['state'] = 'stopped'
        atomic(state, dict(value, heartbeat=time.monotonic()))
        ready.unlink(missing_ok=True)
        if ros is not None:publish_sources(False);ros.close()
        lock.close()
    return 0 if stopped else 1


def observe_owner(state, args):
    """Register the existing raw ROS publisher without owning its hardware lifecycle."""
    from robot_workbench.ros_data import RosNode, descriptor_message
    if not args.ros_namespace or not args.device_id:
        raise ValueError('existing source registration requires namespace and device ID')
    expected=profile(args);source=read_owner(state,expected);publishers={};stopped=False
    ros=RosNode('existing_camera_sources',namespace=args.ros_namespace,
        domain_id=int(expected['ros_domain']),localhost_only=False)
    def declare(available):
        for camera,serial in expected['cameras'].items():
            if camera not in publishers:
                publishers[camera]=ros.publisher('sources/'+camera,ros.types.SourceDescriptor,retained=True,depth=1)
            publishers[camera].publish(descriptor_message(ros,dict(device_id=args.device_id,stream_id=camera,
                source_instance_id=source['instance'],source_clock_id='host_wall',
                topic=expected['topic_prefix']+'/'+camera+'_rgb/image_raw',message_type='sensor_msgs/msg/Image',
                kind='image',units='rgb8',coordinate_frame=camera,nominal_hz=float(expected['fps']),available=available,
                metadata=dict(serial=serial,width=expected['width'],height=expected['height'],environment='physical',
                    timestamp_basis='publisher_host_after_capture'))))
    def stop(*_):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,stop)
    try:
        while not stopped:
            current=read_owner(state,expected)
            if current['instance']!=source['instance']:
                raise ValueError('shared camera publisher changed; explicitly rebind the source')
            declare(current['state']=='running')
            time.sleep(.5)
    finally:
        declare(False);ros.close()
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', required=True)
    parser.add_argument('--cameras', required=True)
    parser.add_argument('--topic_prefix', default='/prometheus/sensors/cameras')
    parser.add_argument('--ros-namespace',dest='ros_namespace')
    parser.add_argument('--device-id',dest='device_id')
    parser.add_argument('--observe-existing',action='store_true',help='register existing raw ROS images; never opens or stops cameras')
    for key, default in dict(width=640, height=480, ir_width=640, ir_height=480, fps=30, qos_depth=10).items():
        parser.add_argument('--' + key, type=int, default=default)
    parser.add_argument('--enable_ir', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    raise SystemExit((observe_owner if args.observe_existing else run_owner)(args.state, args))
