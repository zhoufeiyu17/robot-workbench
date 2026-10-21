"""Cross-runtime launch boundary: clean parent leakage, then apply declared inputs.

Only stdlib supervisors use HOST_PYTHON. Workers keep their explicit interpreter
or runtime launcher (Isaac python.sh, detector venv, etc.). No parent env changes.
"""
import os
from pathlib import Path

HOST_PYTHON='/usr/bin/python3'
HOST_PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
RUNTIME_KEYS=('PYTHONHOME','PYTHONPATH','PYTHONUSERBASE','PYTHONSTARTUP','PYTHONEXECUTABLE','PYTHONPLATLIBDIR',
    '_PYTHON_SYSCONFIGDATA_NAME','_PYTHON_SYSCONFIGDATA_PATH','VIRTUAL_ENV','CONDA_PREFIX','CONDA_DEFAULT_ENV',
    'LD_LIBRARY_PATH','LD_PRELOAD','DYLD_LIBRARY_PATH','DYLD_FALLBACK_LIBRARY_PATH','DYLD_INSERT_LIBRARIES',
    'CARB_APP_PATH','EXP_PATH','ISAAC_PATH','ISAAC_SIM_PATH','ISAACSIM_PATH','PXR_PLUGINPATH_NAME',
    'CUDA_HOME','CUDA_PATH','CUDACXX')


def clean_environment(overrides=None,parent=None):
    value=dict(os.environ if parent is None else parent)
    for key in RUNTIME_KEYS:value.pop(key,None)
    value['PATH']=HOST_PATH
    value.update(overrides or {})
    return value


def clean_command(argv,overrides=None):
    """Use env before Python startup; safe even when caller only accepts argv."""
    return ['/usr/bin/env',*[part for key in RUNTIME_KEYS for part in ('-u',key)],
            'PATH='+HOST_PATH,*[key+'='+str(value) for key,value in (overrides or {}).items()],*map(str,argv)]


def owned_command(workbench,command,timeout,*,failure_result=None):
    extra=[]
    if failure_result is not None:
        if not Path(failure_result).is_absolute():raise ValueError('absolute native failure result required')
        extra=['--failure-result',str(failure_result)]
    lifetime=['--persistent'] if timeout is None else ['--timeout',str(timeout)]
    return clean_command([HOST_PYTHON,'-I','-B',str(Path(workbench)/'owned_process.py'),
                          *lifetime,*extra,'--',*command])


def persistent_service(resources):
    """Missing budgets never opt into an unbounded service implicitly."""
    value=resources.get('persistent_service',False)
    if type(value) is not bool:raise ValueError('persistent_service must be an explicit boolean')
    if value and ('service_lifetime_seconds' not in resources or resources['service_lifetime_seconds'] is not None
            or resources.get('resident_deadline_wall_ns') is not None):
        raise ValueError('persistent service requires explicit null lifetime and no resident deadline')
    return value
