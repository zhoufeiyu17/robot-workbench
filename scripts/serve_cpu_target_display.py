#!/usr/bin/env python3
"""Independent target display: model files and requested poses only, no device IO."""
from pathlib import Path
import argparse,json,math,os,secrets,signal,sys,threading,time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--device-id',required=True);parser.add_argument('--descriptor',type=Path,required=True)
    args=parser.parse_args()
    from arx_env.joint_head_workbench_geometry import Geometry
    from examples.scripts.debug.joint_head_workbench_live import Session,serve
    from robot_workbench.cpu_preview import MeshRenderer
    config=json.loads(args.config.read_text());geometry=Geometry(config)
    model_limits=geometry.limits.copy()
    # Display angles are kinematic inputs, not robot command admission. Preserve
    # tiny boundary-noise values and all SDK targets without changing SDK limits.
    geometry.limits[:]=[-2*math.pi,2*math.pi]
    args.output.mkdir(mode=0o700,parents=True,exist_ok=True)
    token=secrets.token_urlsafe(40);token_file=args.output/'token'
    with os.fdopen(os.open(token_file,os.O_CREAT|os.O_WRONLY|os.O_TRUNC,0o600),'w')as stream:stream.write(token)
    display_config=dict(config,display_only=True,mesh_rendering_enabled=False)
    display_config.pop('wrist_simulation_cameras',None)  # No actual/simulated camera setup in a target-only service.
    session=Session(geometry,args.output/'session',display_config)
    renderer=MeshRenderer(geometry);closed=threading.Event();server=serve(session,token,0)
    for signum in(signal.SIGTERM,signal.SIGINT):signal.signal(signum,lambda *_:closed.set())
    descriptor=dict(schema='workbench.cpu_target_display.v1',device_id=args.device_id,pid=os.getpid(),
        url='http://127.0.0.1:'+str(server.server_port),token_file=str(token_file),renderer_instance_id=session.renderer_instance_id,
        joint_limits=model_limits.tolist(),display_angle_bound_rad=2*math.pi,display_only=True,robot_commands=False)
    temporary=args.descriptor.with_name(args.descriptor.name+'.tmp')
    with os.fdopen(os.open(temporary,os.O_CREAT|os.O_WRONLY|os.O_TRUNC,0o600),'w')as stream:json.dump(descriptor,stream)
    os.replace(temporary,args.descriptor)
    try:
        while not closed.is_set():session.tick(renderer,.02);closed.wait(.01)
    finally:server.shutdown();server.server_close()

if __name__=='__main__':main()
