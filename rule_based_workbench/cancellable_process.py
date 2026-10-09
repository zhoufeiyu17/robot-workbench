"""Run one owned subprocess, allowing a Playground stage to stop it."""
import subprocess
import time


def run(argv, *, cancel=None, timeout=None, check=False, capture_output=False, **kwargs):
    if cancel is None:
        return subprocess.run(argv, timeout=timeout, check=check, capture_output=capture_output, **kwargs)
    if capture_output:
        kwargs.update(stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    started=time.monotonic()
    process=subprocess.Popen(argv, **kwargs)
    try:
        while True:
            if cancel():
                process.terminate()
                try: process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill();process.communicate()
                raise InterruptedError('当前阶段已中断')
            if timeout is not None and time.monotonic()-started>timeout:
                process.kill();process.communicate()
                raise subprocess.TimeoutExpired(argv,timeout)
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
            process.kill();process.communicate()
