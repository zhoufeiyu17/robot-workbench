"""Spawn-only physical execution orchestration; the Owner keeps its own GIL and safety gates."""
import ctypes
import json
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import threading
import time


def _worker(channel, cancelled, parent_pid, config, output, bridge_config, prepared):
    # This happens before importing the pipeline or constructing any transport.
    linux=sys.platform.startswith('linux')
    if linux:
        libc=ctypes.CDLL(None,use_errno=True)
        if libc.prctl(1,signal.SIGKILL,0,0,0)!=0:
            raise OSError(ctypes.get_errno(),'could not bind worker lifetime to its Owner parent')
    if os.getppid()!=parent_pid:return
    signal.signal(signal.SIGTERM,lambda *_:cancelled.set())
    start_ticks=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19] if linux else None
    channel.send(dict(kind='ready',pid=os.getpid(),parent_pid=parent_pid,pdeathsig=linux,
                      start_ticks=start_ticks,output=str(Path(output).resolve())))
    deadline=time.monotonic()+10
    while not channel.poll(.05):
        if cancelled.is_set() or os.getppid()!=parent_pid or time.monotonic()>deadline:return
    if channel.recv()!='start' or os.getppid()!=parent_pid or cancelled.is_set():return
    # Non-Linux CPU test hosts need the same no-orphan property. Linux's kernel
    # guard remains authoritative even when Python is blocked or starved.
    def parent_watch():
        while os.getppid()==parent_pid:time.sleep(.05)
        os._exit(125)
    threading.Thread(target=parent_watch,daemon=True).start()
    from real_bridge import RealBridge
    import playground_pipeline as pipeline
    bridge=None
    try:
        bridge=RealBridge(bridge_config)
        def progress(label,**fields):channel.send(dict(kind='progress',label=label,fields=fields))
        result=pipeline.execute(config,Path(output),bridge,prepared,progress=progress,cancel=cancelled.is_set)
        channel.send(dict(kind='result',value=result))
    except BaseException as error:
        if bridge is not None:pipeline.stop_physical_execution(bridge,output)
        try:channel.send(dict(kind='error',type=type(error).__name__,message=str(error),failure=getattr(error,'failure',None)))
        except (OSError,EOFError):pass
    finally:channel.close()


def execute(config, output, bridge_config, prepared, *, progress=None, cancel=None):
    """One invocation, no restart. Cancellation waits for cleanup, then reaps this worker."""
    if bridge_config.get('environment','physical') not in ('physical','fake'):raise ValueError('explicit physical/fake execution environment required')
    if cancel is not None and cancel():raise InterruptedError('执行准备已中断')
    context=multiprocessing.get_context('spawn')
    channel,child_channel=context.Pipe();cancelled=context.Event()
    process=context.Process(target=_worker,args=(child_channel,cancelled,os.getpid(),config,str(output),bridge_config,prepared),
                            name='physical-execution-orchestrator')
    process.start();child_channel.close()
    ready=False;terminal=None;cancel_deadline=None;startup_deadline=time.monotonic()+15
    try:
        while True:
            if cancel is not None and cancel() and not cancelled.is_set():
                cancelled.set();cancel_deadline=time.monotonic()+30
            if channel.poll(.05):
                try:message=channel.recv()
                except EOFError:break
                kind=message.get('kind')
                if kind=='ready':
                    if ready or message['pid']!=process.pid or message['parent_pid']!=os.getpid():
                        raise RuntimeError('execution worker handshake identity differs')
                    ready=True
                    Path(output).mkdir(parents=True,exist_ok=True)
                    (Path(output)/'execution-worker.json').write_text(json.dumps(dict(message,start_method='spawn')))
                    if not cancelled.is_set():channel.send('start')
                    else:break
                elif kind=='progress':
                    if progress:progress(message['label'],**message['fields'])
                elif kind in ('result','error'):
                    terminal=message
                    break
                else:raise RuntimeError('unknown execution worker message')
            if not process.is_alive():
                # Drain a terminal send completed just before the child exited.
                if channel.poll():continue
                break
            if not ready and time.monotonic()>startup_deadline:raise TimeoutError('execution worker handshake timed out')
            if cancel_deadline is not None and time.monotonic()>cancel_deadline:
                raise TimeoutError('execution cancellation cleanup timed out; physical stop remains unverified')
        process.join(5)
        if process.is_alive():raise RuntimeError('execution worker did not exit after its terminal message')
        if terminal is None or process.exitcode!=0:
            if cancelled.is_set():raise InterruptedError('执行进程已中断；停止状态以控制回执为准')
            raise RuntimeError(f'execution worker exited without a valid result (exit={process.exitcode}); no automatic retry')
        if terminal['kind']=='error':
            error=(InterruptedError(terminal['message']) if terminal['type']=='InterruptedError'
                   else RuntimeError(terminal['type']+': '+terminal['message']))
            error.failure=terminal.get('failure')
            raise error
        return terminal['value']
    except BaseException:
        cancelled.set()
        if cancel_deadline is None:cancel_deadline=time.monotonic()+30
        # The parent is alive, so its Owner cannot rely on PDEATHSIG. Request a
        # stop only for the persisted, matching execution; never another Owner.
        from playground_pipeline import stop_physical_execution
        from real_bridge import RealBridge
        try:stop_physical_execution(RealBridge(bridge_config),output)
        except Exception as cleanup_error:
            (Path(output)/'execution-worker-cleanup-error.json').write_text(json.dumps(
                dict(error=str(cleanup_error),physical_stop='unverified',automatic_retry=False)))
        raise
    finally:
        if process.is_alive():
            cancelled.set();process.join(max(0.,(cancel_deadline or time.monotonic()+30)-time.monotonic()))
            if process.is_alive():
                process.terminate();process.join(2)
            if process.is_alive():process.kill();process.join(5)
        channel.close();process.close()
