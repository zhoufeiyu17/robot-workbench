"""Run one owned subprocess, allowing a Playground stage to stop it."""
import subprocess
import time


def run(argv, *, cancel=None, timeout=None, check=False, capture_output=False, terminate_grace_s=None,on_poll=None, **kwargs):
    if terminate_grace_s is not None and not 0<terminate_grace_s<float('inf'):
        raise ValueError('positive finite subprocess termination grace required')
    if cancel is None and terminate_grace_s is None and on_poll is None:
        return subprocess.run(argv, timeout=timeout, check=check, capture_output=capture_output, **kwargs)
    if capture_output:
        kwargs.update(stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    started=time.monotonic()
    process=subprocess.Popen(argv, **kwargs)
    def terminate():
        try:process.terminate()
        except ProcessLookupError:pass
        try:process.communicate(timeout=5 if terminate_grace_s is None else terminate_grace_s)
        except subprocess.TimeoutExpired:
            process.kill();process.communicate()
    try:
        while True:
            if cancel is not None and cancel():
                terminate()
                raise InterruptedError('当前阶段已中断')
            if timeout is not None and time.monotonic()-started>timeout:
                if terminate_grace_s is None:process.kill();process.communicate()
                else:terminate()
                raise subprocess.TimeoutExpired(argv,timeout)
            if on_poll is not None:on_poll()
            try:
                out,err=process.communicate(timeout=.1)
                break
            except subprocess.TimeoutExpired:
                continue
        result=subprocess.CompletedProcess(argv,process.returncode,out,err)
        if check:result.check_returncode()
        return result
    finally:
        if process.poll() is None:
            if terminate_grace_s is None:process.kill();process.communicate()
            else:terminate()
