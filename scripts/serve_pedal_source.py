#!/usr/bin/env python3
"""One evdev reader publishing canonical ROS pedal events; never opens SDK/CAN."""
import argparse,errno,fcntl,hashlib,json,math,os,signal,sys,threading,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def read_reconnecting(reader,stopped,disconnected):
    """Reopen the configured evdev path; keep the logical ROS source alive."""
    while not stopped.is_set():
        try:
            reader()._run_evdev_backend()
            return
        except (OSError,RuntimeError)as error:
            cause=error.__cause__ if isinstance(error,RuntimeError) else error
            if not isinstance(cause,OSError)or cause.errno not in(errno.ENODEV,errno.ENOENT,errno.ENXIO,errno.EIO):raise
            disconnected()
            if stopped.wait(.25):return


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True);parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();profile=json.loads(args.profile.read_text())
    if set(profile)!={'device_id','data_plane','device','reader_root','reader_sha256','rate_hz','heartbeat_hz'}:
        raise ValueError('exact explicit pedal source configuration required')
    if any(type(profile[k])not in(int,float)or not math.isfinite(profile[k])or profile[k]<=0 for k in ('rate_hz','heartbeat_hz')):
        raise ValueError('finite positive pedal polling/heartbeat rates required')
    reader=Path(profile['reader_root'])/'prometheus/input/pedal_publisher.py'
    if hashlib.sha256(reader.read_bytes()).hexdigest()!=profile['reader_sha256']:raise ValueError('registered pedal reader changed')
    if not Path(profile['device']).is_absolute()or not args.root.is_absolute():raise ValueError('absolute pedal device/root required')
    args.root.mkdir(parents=True,exist_ok=True);lock=(args.root/'source.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    sys.path.insert(0,profile['reader_root'])
    from prometheus.input.pedal_publisher import PedalCommandPublisher,PedalMapping,PedalKeyFilter
    from robot_workbench.ros_data import RosNode,StreamRef,descriptor_message,json_text
    from robot_workbench.managed_services import process_identity
    import inspect
    identity=process_identity(os.getpid());instance=':'.join(str(identity[k])for k in ('pid','start_ticks','boot_id'))
    plane=profile['data_plane'];ros=RosNode('pedal_source_'+str(os.getpid()),namespace=plane['namespace'],domain_id=plane['domain_id'],localhost_only=False)
    publisher=ros.publisher('inputs/pedal',ros.types.DataEvent);declared=ros.publisher('sources/pedal',ros.types.SourceDescriptor,retained=True,depth=1)
    source=dict(device_id=profile['device_id'],stream_id='pedal',source_instance_id=instance,source_clock_id='host_wall',
        topic=publisher.topic_name,message_type='workbench_interfaces/msg/DataEvent',kind='event',units='pedal_command',
        coordinate_frame='operator',nominal_hz=float(profile['heartbeat_hz']),available=False,
        metadata=dict(continuous_window=True,reader_sha256=profile['reader_sha256'],confirm_presses=1,
            timestamp_basis='publisher_receipt; original_evdev_wall_time_in_occurred_at'))
    stopped=threading.Event();connected=threading.Event();mutex=threading.Lock();sequence=[0]
    def declare(available):
        source['available']=available;declared.publish(descriptor_message(ros,source))
        temporary=args.root/'source.tmp';temporary.write_text(json.dumps(source)+'\n');temporary.chmod(0o600);temporary.replace(args.root/'source.json')
    def publish(command=None,*,occurred_at=None):
        if not source['available']:return
        occurred_at=time.time()if occurred_at is None else occurred_at
        with mutex:
            sequence[0]+=1;ref=StreamRef('pedal',instance,'host_wall',time.time_ns(),sequence[0])
            publisher.publish(ros.types.DataEvent(reference=ref.message(),run_id='',epoch=0,
                event='source_tick'if command is None else'pedal_command',
                payload_json=json_text(dict(connected=connected.is_set())if command is None else dict(command=int(command),occurred_at=occurred_at))))
    class Reader(PedalCommandPublisher):
        def __init__(self):
            # Reuse the pinned evdev loop and filter without its second ROS publisher.
            self.node=ros.node;self.device=profile['device'];self.mapping=PedalMapping();self.rate_hz=profile['rate_hz']
            defaults=inspect.signature(PedalKeyFilter).parameters
            self.confirm_presses=1;self.confirm_window_s=defaults['confirm_window_s'].default;self.debounce_s=defaults['debounce_s'].default
            self.rclpy=SimpleNamespace(ok=lambda:not stopped.is_set(),spin_once=lambda *a,**k:None)
        def publish_command(self,cmd,*,occurred_at=None):publish(cmd,occurred_at=occurred_at)
        @staticmethod
        def _evdev_event_type(value):
            # A held key's repeat after USB reconnect is not a new press.
            return 'up'if value==0 else'down'if value==1 else''
        def _mark_ready(self):
            connected.set()
            if not source['available']:declare(True)
            publish()
    def disconnected():
        connected.clear();publish()
    def heartbeat():
        while not stopped.wait(1/profile['heartbeat_hz']):publish()
    # The logical event source stays available while its USB connection waits.
    declare(True)
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.set())
    thread=threading.Thread(target=heartbeat,daemon=True);thread.start()
    try:read_reconnecting(Reader,stopped,disconnected)
    finally:stopped.set();thread.join(2);declare(False);ros.close();lock.close()


if __name__=='__main__':main()
