#!/usr/bin/env python3
"""One deployment-owned backend attempt; each start gets a preserved output root."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec',type=Path,required=True)
    args=parser.parse_args();spec=json.loads(args.spec.read_text())
    if (not isinstance(spec,dict) or spec.get('schema')!='robot_workbench.backend_launch.v1'
            or not isinstance(spec.get('output_root'),str) or not Path(spec['output_root']).is_absolute()
            or not isinstance(spec.get('argv'),list) or not spec['argv']
            or not all(isinstance(x,str) and x and '\0' not in x for x in spec['argv'])
            or not Path(spec['argv'][0]).is_absolute() or spec['argv'].count('{output}')!=1):
        raise ValueError('trusted argv with one output placeholder required')
    env=spec.get('env',{});cwd=spec.get('cwd')
    if (not isinstance(env,dict) or any(not isinstance(key,str) or not key or '=' in key or '\0' in key
            or not isinstance(value,str) or '\0' in value for key,value in env.items())
            or cwd is not None and (not isinstance(cwd,str) or not Path(cwd).is_absolute() or '\0' in cwd)):
        raise ValueError('trusted environment and absolute working directory required')
    if sys.platform!='linux' or not Path('/proc/self/stat').is_file():
        raise RuntimeError('Linux /proc is required before starting the backend')
    root=Path(spec['output_root']);root.mkdir(parents=True,exist_ok=True)
    output=root/('attempt-'+str(time.time_ns()))
    argv=[str(output) if x=='{output}' else x for x in spec['argv']]
    pending=None;child=None
    def stop(signum,_frame):
        nonlocal pending
        pending=signum
        if child is not None and child.poll() is None:
            try:child.send_signal(signum)
            except ProcessLookupError:pass
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    if pending is not None:return 128+pending
    log=root/(output.name+'.log')
    with log.open('xb') as stream:
        child=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,
            env=dict(os.environ,**env),cwd=cwd)
        identity=dict(pid=child.pid,start_ticks=None,
            output=str(output),log=str(log),argv=argv,started_wall_ns=time.time_ns())
        failure=None
        temporary=root/(output.name+'-current.tmp')
        try:
            if pending is not None:stop(pending,None)
            try:identity['start_ticks']=(Path('/proc')/str(child.pid)/'stat').read_text().rsplit(')',1)[1].split()[19]
            except FileNotFoundError:
                if child.poll() is None:raise
                identity['identity_status']='exited_before_identity'
            # Stable pointer is metadata, never a motion-resume request.
            temporary.write_text(json.dumps(identity,indent=2)+'\n');temporary.replace(root/'current.json')
            while child.poll() is None and pending is None:
                try:child.wait(timeout=.1)
                except subprocess.TimeoutExpired:pass
            if child.poll() is None:child.wait(timeout=5)
        except BaseException as error:failure=error
        finally:
            if child.poll() is None:
                try:
                    child.terminate();child.wait(timeout=5)
                except (OSError,subprocess.TimeoutExpired) as error:identity['cleanup_error']=str(error)
            result=child.poll()
            identity.update(exit_code=result,status='unknown' if result is None else 'exited',finished_wall_ns=time.time_ns())
            if failure is not None:identity['error']=str(failure)
            try:(root/(output.name+'-exit.json')).write_text(json.dumps(identity,indent=2)+'\n')
            except OSError as error:
                if failure is None:failure=error
            try:temporary.unlink(missing_ok=True)
            except OSError:pass
        if failure is not None:raise failure
        if result is None:raise RuntimeError('backend exit remains unknown; no KILL was sent')
        return 128-result if result<0 else result


if __name__=='__main__':raise SystemExit(main())
