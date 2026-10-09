#!/usr/bin/env python3
"""Prepare D from installed C and a live frozen resident; never command motion.

Default: CPU preparation and a conversion recipe. --run additionally converts
assets and checks private worker readiness, then closes those new workers. The
parent's code, installation, controller and original deadline are read-only.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]


def read(path):return json.loads(Path(path).read_bytes())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):Path(path).write_text(json.dumps(value,allow_nan=False,indent=2)+'\n')


def live_parent(root,contract,*,proc_root='/proc',now_ns=None):
    from local_execution import TARGET,validate
    root=Path(root).resolve();service=read(root/'service.json');run=read(root/'run.json')
    pid=service.get('pid')
    if type(pid) is not int or pid!=run.get('pid') or service.get('environment')!='fake':
        raise ValueError('same live fake resident PID required')
    proc=Path(proc_root)/str(pid)
    argv=[part.decode() for part in (proc/'cmdline').read_bytes().split(b'\0') if part]
    def argument(name):
        if argv.count(name)!=1 or argv.index(name)+1>=len(argv):raise ValueError('parent argv missing '+name)
        return argv[argv.index(name)+1]
    script=next((value for value in argv if Path(value).name=='serve_physx_device.py'),None)
    if not script or not Path(script).is_absolute() or Path(argument('--output')).resolve()!=root:
        raise ValueError('PID is not this frozen resident output')
    original=read(argument('--collection-contract'))
    session=contract['identity']['session_id']
    if original['identity']['session_id']!=session or contract['resources'].get('max_workers')!=4:
        raise ValueError('setup-only keeps the original Session and four-role deployment')
    deadline=service.get('resident_deadline_wall_ns')
    if (type(deadline) is not int or deadline!=run.get('resident_deadline_wall_ns')
            or deadline-(time.time_ns() if now_ns is None else now_ns)<=60_000_000_000):
        raise ValueError('original resident deadline has no setup/cleanup time; it cannot be extended')
    base_path=Path(argument('--base'))
    if sha(base_path)!=run['base_sha256']:raise ValueError('parent base changed after startup')
    details=contract['collection_setup']
    execution=dict(TARGET,session_id=session,max_processes=4,min_free_memory_mib=details['min_free_memory_mib'],
        plant=dict(session_id=session,pid=pid,process_start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19],
            scope=str(root),script=script,script_sha256=sha(script),argv=argv,max_memory_mib=details['plant_max_memory_mib']))
    validate({'execution':execution})
    manifest_path=root/'calibrations/hardware/active/manifest.json';manifest=read(manifest_path)
    if manifest.get('simulation_only') is not True or manifest.get('robot_source')!='simulated':
        raise ValueError('current installed fake C group required')
    expected=dict(head_id=manifest['head']['id'],wrist_id=manifest['wrist']['id'],manifest_sha256=sha(manifest_path))
    frozen={str(path):sha(path) for path in (root/'service.json',root/'run.json',root/'calibration-config.json',
        root/'registration.json',manifest_path,Path(script),base_path)}
    return dict(service=service,execution=execution,installed=expected,deadline=deadline,
                base=read(base_path),frozen=frozen)


def verify_parent(parent,*,proc_root='/proc'):
    for path,digest in parent['frozen'].items():
        if sha(path)!=digest:raise ValueError('read-only parent evidence changed: '+path)
    plant=parent['execution']['plant'];proc=Path(proc_root)/str(plant['pid'])
    if (proc/'stat').read_text().rsplit(')',1)[1].split()[19]!=plant['process_start_ticks']:
        raise ValueError('resident PID was reused')
    argv=[value.decode() for value in (proc/'cmdline').read_bytes().split(b'\0') if value]
    if argv!=plant['argv']:raise ValueError('resident process command changed')
    if time.time_ns()>=parent['deadline']-45_000_000_000:raise TimeoutError('original resident deadline reached')


def run(args):
    from robot_workbench.resident_collection import load_collection_contract
    from robot_workbench.collection_setup import prepare_collection_setup,finalize_collection_setup
    from launch_environment import owned_command,clean_command,clean_environment
    from local_execution import gpu_check
    from run_pipeline import load_config
    root=args.resident.resolve();output=args.output.resolve()
    if output==root or not output.is_relative_to(root) or output.exists():
        raise ValueError('a new output directory strictly inside the current resident scope is required')
    # Validate policy before any output or process creation. The original deadline
    # is read from the resident; this command cannot create a new lifetime.
    contract=load_collection_contract(args.contract,output,1,0)
    parent=live_parent(root,contract);service=parent['service'];execution=parent['execution']
    from detector_adapter.preflight_release import preflight_release
    release=preflight_release(parent['base']['pipeline_config'],
        contract['collection_setup']['detection_runtime_root'])
    output.mkdir(mode=0o700);(output/'raw-unused').mkdir()
    save(output/'parent-readonly.json',dict(execution=execution,installed=parent['installed'],
        frozen=parent['frozen'],resident_deadline_wall_ns=parent['deadline'],movement_allowed=False,
        detector_release_preflight=release))
    base=deepcopy(parent['base']['pipeline_config'])
    base['detection_script']=str(ROOT/'detector_adapter/isaac_rgbd_detection.py')
    settings=contract['collection_setup']
    setup=prepare_collection_setup(run_root=root,calibration_config=root/'calibration-config.json',
        capture_root=root/'calibration-workbench',expected_install=parent['installed'],registration=root/'registration.json',
        profile_template=settings['profile_template'],base_pipeline_config=base,execution=execution,
        contract_template=contract,detection_runtime_root=settings['detection_runtime_root'],
        control_url='http://127.0.0.1:'+str(service['gateway_port']),token_file=service['gateway_token_files']['collection'],
        producer_ports=service['camera_ports'],producer_directories=service['camera_directories'],
        trusted_binding_path=output/'unused-control-binding.json',local_recording_root=output/'raw-unused',
        resident_deadline_wall_ns=parent['deadline'],output=output/'configuration')
    result=dict(status='prepared_no_gpu_started',setup=setup,movement_allowed=False,parent_modified=False,
                resident_deadline_wall_ns=parent['deadline'],gpu_qualification=False)
    save(output/'result.json',result)
    if not args.run:verify_parent(parent);return result
    detector=planner=None;process=None
    try:
        verify_parent(parent);gpu_check({'execution':execution})
        conversion=setup['conversion'];budget=min(600.,(parent['deadline']-time.time_ns())/1e9-60)
        if budget<=0:raise TimeoutError('no original resident budget remains for conversion')
        command=owned_command(ROOT/'rule_based_workbench',clean_command(conversion['argv'],conversion['env']),budget)
        with (output/'conversion.log').open('xb') as stream:
            process=subprocess.Popen(command,env=clean_environment(),stdout=stream,stderr=subprocess.STDOUT)
            code=process.wait(timeout=budget+30)
            if code:raise RuntimeError('conversion failed; inspect this new output/conversion.log')
        verify_parent(parent)
        configured=finalize_collection_setup(setup['setup_path'])
        if args.conversion_only:
            result.update(status='conversion_verified_workers_not_started',configured=configured,motion_started=False,task_qualified=False)
            return result
        cfg,_,_=load_config(Path(configured['pipeline_config']))
        from detector_service import DetectorService
        from planner_service import PlannerService
        detector,planner=DetectorService(),PlannerService()
        for name,worker in (('detector',detector),('planner',planner)):
            verify_parent(parent)
            worker.start(configured['pipeline_config'],configured['worker_state_directory'])
            end=min(time.monotonic()+args.readiness_seconds,time.monotonic()+(parent['deadline']-time.time_ns())/1e9-45)
            while time.monotonic()<end:
                state=worker.status()
                save(output/(name+'-readiness.json'),state)
                if state.get('status')=='ready':break
                if state.get('status') in ('failed','stopped'):raise RuntimeError(name+' did not become ready: '+str(state))
                time.sleep(.2)
            else:raise TimeoutError(name+' readiness exceeded its original resident budget')
        gpu_check(cfg,require='detector');gpu_check(cfg,require='planner');verify_parent(parent)
        result.update(status='conversion_and_workers_ready',configured=configured,
            detector=detector.status(),planner=planner.status(),motion_started=False,
            task_qualified=False,note='readiness only; private workers close on exit; no control binding is published')
    except BaseException as error:
        result.update(status='failed',error=f'{type(error).__name__}: {error}');raise
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        failures=[]
        for worker in (planner,detector):
            if worker:
                try:worker.close()
                except Exception as error:failures.append(str(error))
        result['worker_cleanup_errors']=failures;result['new_workers_closed']=not failures
        save(output/'result.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resident',type=Path,required=True)
    parser.add_argument('--contract',type=Path,required=True,help='new explicit contract retaining the original Session/resource scope')
    parser.add_argument('--output',type=Path,required=True,help='new directory inside resident scope')
    parser.add_argument('--run',action='store_true',help='explicitly run conversion + worker readiness only; never motion')
    parser.add_argument('--conversion-only',action='store_true',help='with --run, verify conversion and stop before any detector/planner startup')
    parser.add_argument('--readiness-seconds',type=float,default=180.)
    args=parser.parse_args()
    if args.conversion_only and not args.run:parser.error('--conversion-only requires explicit --run')
    if not 0<args.readiness_seconds<=600:parser.error('bounded readiness seconds required')
    print(json.dumps(run(args),allow_nan=False))
