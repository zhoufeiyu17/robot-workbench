"""CPU-only service ownership, GPU admission and lifecycle checks."""
import io
import json
from pathlib import Path
import shlex
from tempfile import TemporaryDirectory
from unittest.mock import patch, Mock
from detector_service import DetectorService, OWNED_GPU_CHECK, SUPERVISOR, gpu_check_command, validate_config, launch_command


def main():
    spec={'root':'/remote/service','socket':'/remote/service/s','owner_token':'nonce-01234567890123',
          'identity':{'adapter_sha256':'sha256:a'},'allowed_read_roots':['/remote/jobs'],
          'allowed_output_root':'/remote/jobs','max_memory_mib':16384,'min_free_memory_mib':16384}
    config={'detector_service':spec,'detection_script':'/remote/code/detector.py'}
    from copy import deepcopy
    frozen=deepcopy(config)
    frozen['remote_run_root']='/remote/jobs/run'
    frozen['detection_node_config']='/remote/config/node'
    frozen['detector_service'].update(root='/mnt/data/feiyu.zhou/runs/project/service',socket='/mnt/data/feiyu.zhou/runs/project/service/s',max_lifetime_seconds=7200,
        identity={'adapter_sha256':'sha256:'+'a'*64,'service_sha256':'sha256:'+'b'*64,'node_config_sha256':'sha256:'+'c'*64})
    contract={'detector_service':deepcopy(frozen['detector_service']),
        'filesystem':{'writable_roots':[frozen['detector_service']['root']], 'readonly_roots':['/remote']},
        'resources':{'max_workers_per_instance':2},
        'provenance':{'code_sha256':{'/remote/code/detector.py':'a'*64,'/remote/code/isaac_detection_service.py':'b'*64},'input_sha256':{'/remote/config/node':'c'*64}}}
    validate_config(frozen,contract)
    for lifetime in (0, 7200, 86400):
        c=deepcopy(frozen);t=deepcopy(contract)
        c['detector_service']['max_lifetime_seconds']=lifetime
        t['detector_service']=deepcopy(c['detector_service'])
        validate_config(c,t)
    for lifetime in (-1, True, None):
        c=deepcopy(frozen);t=deepcopy(contract)
        c['detector_service']['max_lifetime_seconds']=lifetime
        t['detector_service']=deepcopy(c['detector_service'])
        try:validate_config(c,t)
        except ValueError:pass
        else:raise AssertionError('invalid lifetime accepted')
    # Remote services require an explicit contract opt-in; missing deadlines alone never opt in.
    c=deepcopy(frozen);t=deepcopy(contract)
    t['resources'].update(persistent_service=True,service_lifetime_seconds=None)
    c['detector_service'].update(persistent_service=True,max_lifetime_seconds=None)
    t['detector_service']=deepcopy(c['detector_service'])
    validate_config(c,t)
    for resources in ({'persistent_service':False,'service_lifetime_seconds':None},
                      {'persistent_service':True},
                      {'persistent_service':True,'service_lifetime_seconds':7200}):
        bad=deepcopy(t);bad['resources']={'max_workers_per_instance':2,**resources}
        try:validate_config(c,bad)
        except ValueError:pass
        else:raise AssertionError('unapproved remote persistence accepted')
    c.update(detection_runtime_root='/remote/runtime',detection_python='/remote/venv/bin/python',
             detection_cuda_home='/remote/cuda')
    t['provenance']['code_sha256']['/remote/runtime/scripts/run_online_detection_episode.py']='d'*64
    with TemporaryDirectory() as temp:
        c['contract_path']=str(Path(temp)/'contract.json');Path(c['contract_path']).write_text(json.dumps(t))
        command=shlex.split(launch_command(c));assert command[-1]=='persistent'
        worker=json.loads(command[-2])
        assert 'ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT=/remote/runtime' in worker
        assert any(value.startswith('ROBOT_WORKBENCH_DETECTOR_RUNTIME_IDENTITY=') for value in worker)
        assert 'CUDA_VISIBLE_DEVICES=0' in worker and 'execution' not in c
    for service_root, allowed in [('/tmp/rbw-detector-20260923-0310/service-acde',True),('/tmp/unrelated/service',False),('/tmp/rbw-detector-/service',False),('/tmp/rbw-detector-test',False)]:
        tmp_config=deepcopy(frozen);tmp_contract=deepcopy(contract)
        tmp_config['detector_service'].update(root=service_root,socket=service_root+'/s')
        tmp_contract['detector_service']=deepcopy(tmp_config['detector_service'])
        tmp_contract['filesystem']['writable_roots']=[service_root]
        try:validate_config(tmp_config,tmp_contract)
        except ValueError:assert not allowed
        else:assert allowed
    bad=deepcopy(contract);bad['detector_service']['owner_token']='foreign'
    try:validate_config(frozen,bad)
    except ValueError:pass
    else:raise AssertionError('unfrozen service accepted')
    state={**spec,'service_root':spec['root'],'gpu':0,'status':'ready','pid':123,'process_start_ticks':999}
    state.pop('root')
    def read_text(p,*args,**kwargs):
        if p.name=='status.json':return json.dumps(state)
        if p.name=='stat':return '123 (detector with spaces) '+' '.join(['S']+['0']*18+['999']+['0']*3)
        raise AssertionError(str(p))
    def read_bytes(p,*args,**kwargs):
        return b'python\0/remote/code/detector.py\0--serve\0--owner-token\0nonce-01234567890123\0--service-root\0/remote/service\0'
    rows=['GPU-a, 123, 2048\n']
    def output(args,**kwargs):return 'GPU-a, 65000\n' if '-i' in args else rows[0]
    def check():
        with patch('pathlib.Path.read_text',read_text),patch('pathlib.Path.read_bytes',read_bytes),patch('subprocess.check_output',output),patch('sys.argv',['check',json.dumps(config)]):
            exec(compile(OWNED_GPU_CHECK,'gpu_check','exec'),{})
    check()
    def rejected():
        try:check()
        except AssertionError:return
        raise AssertionError('unsafe admission accepted')
    rows[0]+='GPU-a, 987, 1024\n';rejected();rows[0]='GPU-a, 123, 2048\n'
    state['process_start_ticks']=1000;rejected();state['process_start_ticks']=999
    state['status']='busy';rejected();state['status']='ready'
    state['owner_token']='foreign';rejected();state['owner_token']=spec['owner_token']
    rows[0]='GPU-a, 123, 20000\n';rejected()
    assert 'selected GPU is not idle' in gpu_check_command(config,'dsw-scene')
    assert 'detector PID was reused' in gpu_check_command(config,'simulated-real')
    service=DetectorService(); process=Mock();process.stdin=io.StringIO();service._process=process
    service.close();assert process.stdin.closed;process.wait.assert_called_once_with(timeout=30)
    process.terminate.assert_not_called();assert service.status()['status']=='stopped'
    # Supervisor EOF terminates only the Popen child it created, including that owned group.
    for lifetime in ('0','persistent'):
        child=Mock(pid=456,returncode=0);child.poll.return_value=None
        with patch('subprocess.Popen',return_value=child) as spawn,patch('select.select',return_value=([0],[],[])),patch('os.read',side_effect=[b'x',b'']),patch('os.killpg') as kill,patch('sys.argv',['supervisor','["detector"]',lifetime]),patch('time.monotonic',side_effect=AssertionError('unlimited supervisor must not use a deadline')),patch('sys.stdin',Mock()):
            try:exec(compile(SUPERVISOR,'supervisor','exec'),{})
            except SystemExit:pass
            spawn.assert_called_once();assert spawn.call_args.kwargs['start_new_session'] is True
            assert kill.call_args.args[0]==456
        with patch('subprocess.Popen') as spawn,patch('select.select',return_value=([0],[],[])),patch('os.read',return_value=b''),patch('sys.argv',['supervisor','["detector"]',lifetime]),patch('sys.stdin',Mock()):
            try:exec(compile(SUPERVISOR,'supervisor','exec'),{})
            except SystemExit:pass
            spawn.assert_not_called()  # Stop/EOF before the remote launch does not create a model.
    # A structured service failure survives the subsequent SSH exit.
    manager=DetectorService()
    failed={'owner_token':spec['owner_token'],'identity':spec['identity'],'service_root':spec['root'],'socket':spec['socket'],
            'status':'failed','initialization':{'stage':'stopped','error':'[Errno 95] Operation not supported','message':'检测服务已停止'}}
    fake=Mock();fake.stdout=io.StringIO(json.dumps(failed)+'\n');fake.stdin=io.StringIO();fake.wait.return_value=1
    from unittest.mock import patch as patched
    with TemporaryDirectory() as temp,patched('subprocess.Popen',return_value=fake),patched('subprocess.run',return_value=Mock(returncode=0)),patched('detector_service.launch_command',return_value='fake'):
        manager._run(config,{'provenance':{}},Path(temp))
    assert manager.status()['error']=='[Errno 95] Operation not supported'
    # Initialization elapsed time freezes at the first ready, including busy/runtime failure.
    manager=DetectorService(); frames=[]
    for status,elapsed in [('starting',10),('ready',150),('busy',300),('ready',400),('failed',500)]:
        frame={**failed,'status':status,'initialization':{'stage':status,'elapsed_s':elapsed,'error':'runtime error' if status=='failed' else None}}
        frames.append(json.dumps(frame))
    fake=Mock();fake.stdout=io.StringIO('\n'.join(frames)+'\n');fake.stdin=io.StringIO();fake.wait.return_value=1
    with TemporaryDirectory() as temp,patched('subprocess.Popen',return_value=fake),patched('subprocess.run',return_value=Mock(returncode=0)),patched('detector_service.launch_command',return_value='fake'):
        manager._run(config,{'provenance':{}},Path(temp))
    assert manager.status()['elapsed_s']==150 and manager.status()['error']=='runtime error'
    # Prior detector shutdown may outlive the workbench. Retry only read-only GPU admission,
    # bounded to 30 s; launch the model exactly once after admission succeeds.
    clock=[0.0]
    def sleep(seconds):clock[0]+=seconds
    busy=Mock(returncode=1,stderr='AssertionError: selected GPU has a compute process')
    manager=DetectorService(); observed=[]
    def admission(*args,**kwargs):
        assert 0<kwargs['timeout']<=30
        observed.append(manager.status())
        return busy if len(observed)<3 else Mock(returncode=0)
    fake=Mock();fake.stdout=io.StringIO('');fake.stdin=io.StringIO();fake.wait.return_value=0
    with TemporaryDirectory() as temp,patched('time.monotonic',side_effect=lambda:clock[0]),patched('time.sleep',side_effect=sleep),patched('subprocess.run',side_effect=admission) as check,patched('subprocess.Popen',return_value=fake) as spawn,patched('detector_service.launch_command',return_value='fake'),patched('os.kill') as kill:
        manager._run(config,{'provenance':{}},Path(temp))
        assert check.call_count==3 and clock[0]==2
        assert '等待 GPU 释放' in observed[1]['message']
        spawn.assert_called_once();kill.assert_not_called()
    clock[0]=0;manager=DetectorService()
    with patched('time.monotonic',side_effect=lambda:clock[0]),patched('time.sleep',side_effect=sleep),patched('subprocess.run',return_value=busy) as check,patched('subprocess.Popen') as spawn:
        try:manager._wait_for_gpu()
        except RuntimeError as error:assert '30 秒' in str(error)
        else:raise AssertionError('busy GPU accepted after deadline')
        assert clock[0]==30 and check.call_count==30;spawn.assert_not_called()
    manager=DetectorService()
    with patched('subprocess.run',return_value=Mock(returncode=255,stderr='SSH authentication failed')) as check,patched('time.sleep') as delay:
        try:manager._wait_for_gpu()
        except RuntimeError as error:assert 'authentication failed' in str(error)
        else:raise AssertionError('SSH failure treated as GPU release')
        check.assert_called_once();delay.assert_not_called()
    manager=DetectorService();manager.close()
    with patched('subprocess.run') as check:
        assert manager._wait_for_gpu() is False;check.assert_not_called()
    print('Detector ownership, busy rejection, unknown PID, PID reuse, memory limit and EOF cleanup: PASS')
    print('GPU release wait: bounded 30 s, visible progress, one model launch, genuine errors and cancellation: PASS')

if __name__=='__main__':main()
