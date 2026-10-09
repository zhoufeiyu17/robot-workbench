"""Prepare a physical RGB-D scene and frozen Isaac plan; never execute robot motion."""
import argparse
import hashlib
import json
import re
from pathlib import Path, PurePosixPath
import shlex
import shutil
import subprocess
import threading
import time

from integration import _digest, validate_profile
from real_contract import validate_real_plan, namespace, validate_namespace
from robot_workbench.sample_clock import clock_for_source, sample_stamp, assert_progress, owner_identity
from run_pipeline import (absolute, below, bounded_command, build_plan, load_config,
                          pinned_files, pull_telemetry, python_command, replay_receipt, save, ssh,
                          start_preview_stream, stop_preview_stream)
from run_pipeline import transport
from live_preview import PREVIEW_CAMERAS, camera_frame_path, receive_stream
from playback import validate_target
from runtime_progress import BASELINE, estimates


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_capture(capture, binding, *, environment='physical'):
    """An accepted acquisition receipt binds the exact physical inputs, not a label."""
    capture = Path(capture)
    profile = json.loads((capture/'profile.json').read_text())
    manifest = json.loads((capture/'rgbd-manifest.json').read_text())
    from real_observation import validate_binding
    validate_binding(profile,binding,environment=environment)
    validate_namespace(binding, environment)
    validate_namespace(manifest, environment)
    clock=clock_for_source(binding,environment)
    if environment=='fake' and (manifest.get('clock')!=clock or manifest.get('source_clock')!='simulation'
                               or manifest.get('source_clock_id')!=clock['source_clock_id']):
        raise ValueError('capture manifest clock differs from sealed fake source clock')
    cameras = binding.get('camera_serials')
    if (binding.get('calibration_accepted') is not True
            or not isinstance(binding.get('robot_id'), str) or not binding['robot_id'].strip()
            or not isinstance(cameras, dict) or set(cameras) != {'head','left','right'}
            or any(not isinstance(v,str) or not v.strip() for v in cameras.values()) or len(set(cameras.values())) != 3
            or binding.get('calibration_profile_digest') != _digest(profile)
            or binding.get('rgbd_manifest_sha256') != file_sha(capture/'rgbd-manifest.json')):
        raise ValueError('accepted physical acquisition identity and exact profile/manifest binding required')
    if (manifest.get('robot_id') != binding['robot_id']
            or manifest.get('camera_serials') != cameras or environment == 'physical' and manifest.get('source_clock') == 'simulation'):
        raise ValueError('capture manifest must identify this physical robot and cameras')
    frames = manifest.get('frames')
    if not isinstance(frames,list) or len(frames) < 6:
        raise ValueError('physical capture requires at least six RGB-D frames spanning five seconds')
    previous_stamp = previous_sequence = -1
    previous_clock=None
    paths=set()
    for row in frames:
        path = PurePosixPath(row.get('path',''))
        if not path.parts or path.is_absolute() or '..' in path.parts or path.suffix != '.npz' or str(path) in paths:
            raise ValueError('capture frame path must be a distinct contained relative NPZ')
        paths.add(str(path))
        frame = capture / path
        if (not frame.is_file() or frame.is_symlink() or not frame.resolve().is_relative_to(capture.resolve())
                or file_sha(frame) != row.get('sha256')):
            raise ValueError('physical RGB-D frame hash differs or path escapes capture')
        stamp, sequence = row.get('source_stamp_ns'), row.get('capture_sequence_id')
        if environment=='fake':
            current_clock=sample_stamp(row,clock,source_key='source_stamp_ns')
            assert_progress(previous_clock,current_clock);previous_clock=current_clock
            if row.get('source_time_ns')!=stamp:raise ValueError('capture source stamps disagree')
            if owner_identity(row)!=binding.get('capture_owner'):
                raise ValueError('capture row Owner differs from the sealed observation identity')
        if type(stamp) is not int or type(sequence) is not int or stamp <= previous_stamp or sequence <= previous_sequence:
            raise ValueError('physical frame timestamps and sequence must increase')
        previous_stamp, previous_sequence = stamp, sequence
    if (frames[-1]['source_stamp_ns']-frames[0]['source_stamp_ns']) < 5_000_000_000:
        raise ValueError('physical capture is shorter than five seconds')
    return manifest


def import_capture(capture, binding_path, destination, environment='physical'):
    capture, destination = Path(capture), Path(destination)
    binding = json.loads(Path(binding_path).read_text())
    manifest = validate_capture(capture, binding, environment=environment)
    destination.mkdir(exist_ok=False)
    # Copy only receipt-bound detector inputs; no unrelated recordings or links.
    for name in ('profile.json','rgbd-manifest.json',*(row['path'] for row in manifest['frames'])):
        target = destination/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(capture/name,target)
    (destination/'acquisition-binding.json').write_text(json.dumps(binding,allow_nan=False))
    validate_capture(destination,binding,environment=environment)


def canonical_sha(value):
    if not isinstance(value, str) or not re.fullmatch(r'(?:sha256:)?[0-9a-f]{64}', value):
        raise ValueError('invalid SHA256 representation')
    return value.removeprefix('sha256:')


def bind_detection(root, environment='physical'):
    root = Path(root)
    binding = json.loads((root/'capture/acquisition-binding.json').read_text())
    manifest = validate_capture(root/'capture',binding,environment=environment)
    summary = json.loads((root/'detection/summary.json').read_text())
    if (canonical_sha(summary.get('manifest_sha256')) != canonical_sha(binding['rgbd_manifest_sha256'])
            or [canonical_sha(row.get('sha256')) for row in summary.get('inputs',[])] != [row['sha256'] for row in manifest['frames']]):
        raise ValueError('detector outputs do not refer to receipt-bound physical RGB-D input')
    binding['detection_files'] = dict(summary_sha256=file_sha(root/'detection/summary.json'),
                                     observations_sha256=file_sha(root/'detection/observations.jsonl'))
    path = root/'physical-detection-binding.json'
    with path.open('x') as stream:
        json.dump(binding,stream,allow_nan=False)


def preparation_stages(config, contract, capture_dir, physical_binding, *, scene_only=False):
    """Bind physical inputs; Playground supplies its own planner after scene setup.

    The full native CLI still owns rollout/export/replay. Its dual-DSW MotionGen
    stages cannot be reused for a physical source.
    """
    environment = namespace(config.get('environment', 'physical'))['environment']
    if config.get('planning_backend') == 'motiongen' and not scene_only:
        raise ValueError('MotionGen physical preparation requires the Playground scene/planning entry')
    capture_dir, physical_binding = absolute(str(capture_dir)), absolute(str(physical_binding))
    if config.get('force_diagnostic'):
        raise ValueError('physical preparation cannot reuse forced diagnostic plans')
    if any(config.get(key) for key in ('observation_input_root','detection_input_root','planned_input_root')):
        raise ValueError('physical preparation requires its sealed capture, not reused simulation inputs')
    render_replay = config.get('render_replay', True)
    if render_replay and (config.get('record_raw') is not True or not config.get('prompt_file')):
        raise ValueError('physical preparation requires raw target recording and a pinned prompt for three-camera replay')
    read_roots = contract['filesystem']['readonly_roots']
    pins = {name:sha for group in pinned_files(contract) for name,sha in group.items()}
    for path in (capture_dir+'/profile.json', capture_dir+'/rgbd-manifest.json', physical_binding):
        if below(path,config['remote_run_root']) or not any(below(path,root) for root in read_roots) or path not in pins:
            raise ValueError('physical capture/profile/binding must be contract-declared and SHA-pinned readonly inputs')
    stages = build_plan(config,contract)
    root, wb = config['remote_run_root'],config['workbench_code']

    def stage(name,side,command,output=None):
        return dict(name=name,side=side,timeout_s=600,group=None,output=output,
                    argv=transport(config,side,'timeout -k 30 600 '+shlex.join(['bash','-c',command]),timeout=600))

    def cpu(script,*args):
        return python_command('import sys; sys.path[:0] = '+repr([wb,str(PurePosixPath(wb).parent)])+';\n'+script, *args)

    result=[]
    for item in stages:
        name=item['name']
        if name == 'observe':
            result.append(stage('physical-capture-import','simulated-real',cpu(
                'from real_pipeline import import_capture; import sys; import_capture(*sys.argv[1:])',
                capture_dir,physical_binding,root+'/capture',environment)))
        elif name == 'bridge':
            result.append(stage('physical-detection-bind','simulated-real',cpu(
                'from real_pipeline import bind_detection; import sys; bind_detection(*sys.argv[1:])',root,environment)))
            # Keep the original environment and bind this physical observation.
            argv=list(item['argv']); wrapped=shlex.split(argv[-1])
            # MotionGen's dual-DSW bridge adds a simulation start; the physical
            # measured start is already sealed by acquisition and must stand alone.
            wrapped[-1] = wrapped[-1].replace(' --simulation-initial-suite '+shlex.quote(config['initial_suite']),'')
            wrapped[-1] += ' --physical-binding '+shlex.quote(root+'/physical-detection-binding.json')
            if environment == 'fake':wrapped[-1] += ' --environment fake'
            argv[-1]=shlex.join(wrapped)
            result.append({**item,'argv':argv})
            result.append(stage('physical-model-bind','dsw-scene',cpu(
                'import json,sys; from a1_asset_validation import validate_assets; '
                'print(json.dumps(validate_assets(sys.argv[1],sys.argv[2],json.loads(sys.argv[3]))))',
                config['assets'],root+'/capture/profile.json',json.dumps(pins))))
            if scene_only:
                return result
        elif name == 'export':
            argv=list(item['argv']); wrapped=shlex.split(argv[-1])
            old="export(root/'detection-bridge/source.json',root/'place/scene-000'/episode,root/'plan.json')"
            new="export(root/'detection-bridge/source.json',root/'place/scene-000'/episode,root/'plan.json',hardware_binding=root/'detection-bridge/hardware-binding.json',environment="+repr(environment)+")"
            # The nested shell command is tokenized to avoid replacing shell-escaped Python.
            tokens=shlex.split(wrapped[-1]); found=False
            for token in tokens:
                if old in token:
                    wrapped[-1]=wrapped[-1].replace(shlex.quote(token),shlex.quote(token.replace(old,new)),1)
                    found=True
            if not found: raise ValueError('native export invocation changed; physical binding was not applied')
            argv[-1]=shlex.join(wrapped)
            result.append({**item,'argv':argv})
        elif name == 'replay-target' and render_replay:
            argv=list(item['argv']); wrapped=shlex.split(argv[-1])
            wrapped[-1]=wrapped[-1].replace(' --barrier '+shlex.quote(root+'/barrier'),'')
            wrapped[-1]+=' --independent-replay --hardware-binding '+shlex.quote(root+'/detection-bridge/hardware-binding.json')
            if environment == 'fake':wrapped[-1]+=' --environment fake'
            argv[-1]=shlex.join(wrapped)
            result.append({**item,'name':'physical-target-replay','group':None,'argv':argv})
        elif name == 'export-playback' and render_replay:
            result.append(stage('export-target-playback','dsw-scene',shlex.join([
                'env','CUDA_VISIBLE_DEVICES=-1','PYTHONDONTWRITEBYTECODE=1','python3','-B',
                wb+'/playback.py','--run-root',root,'--target-only','--environment',environment])))
        elif name.startswith('preflight-') or name in {'initialize','shared-storage','detect','geometry','geometry-verify','place','fetch-source.json','fetch-plan.json'}:
            result.append(item)
    result.append(stage('fetch-hardware-binding.json','simulated-real',python_command(
        "import pathlib,sys; p=pathlib.Path(sys.argv[1]); assert p.stat().st_size<=1048576; sys.stdout.buffer.write(p.read_bytes())",
        root+'/detection-bridge/hardware-binding.json'),output='hardware-binding.json'))
    for name in (() if not render_replay else ('target-playback/manifest.json',*(f'target-playback/target-{camera}.mp4' for camera in PREVIEW_CAMERAS),
                 'replay-target/execution.json','replay-target/telemetry.jsonl','replay-target/performance.json')):
        result.append(stage('fetch-'+name.replace('/','-'),'dsw-scene',python_command(
            "import pathlib,sys; p=pathlib.Path(sys.argv[1]); assert p.stat().st_size<=32*1024**2; sys.stdout.buffer.write(p.read_bytes())",
            root+'/'+name),output=name))
    return result


def replay_monitor(config,output,stop):
    """Display transport never decides whether the independently timed rollout succeeded."""
    while not stop.is_set():
        pull_telemetry('dsw-scene',config['remote_run_root']+'/replay-target/telemetry.jsonl',output/'telemetry-target.jsonl',config)
        stop.wait(1)


def execute_target(stage,argv,seconds,config,output,log):
    readers=[]
    stop=threading.Event()
    monitor=threading.Thread(target=replay_monitor,args=(config,output,stop),daemon=True)
    try:
        for camera in PREVIEW_CAMERAS:
            path=camera_frame_path(output/'replay-target',camera)
            try:
                process,stream=start_preview_stream(config,'dsw-scene','target',path,camera)
            except (OSError,subprocess.SubprocessError):
                continue
            reader=threading.Thread(target=receive_stream,args=(stream,path),daemon=True) if stream else None
            readers.append((process,stream,reader))
            if reader: reader.start()
        monitor.start()
        completed=subprocess.run(argv,stdout=log,stderr=subprocess.STDOUT,timeout=seconds+45)
        if completed.returncode==0: replay_receipt(config,'dsw-scene','target',output)
        return completed
    finally:
        stop.set()
        if monitor.ident is not None: monitor.join(timeout=13)
        for process,stream,reader in readers:
            stop_preview_stream(process,stream)
            if reader: reader.join(timeout=5)


def run(config_path,output,*,capture_dir,physical_binding,dry_run=False):
    config,contract,hashes=load_config(Path(config_path))
    spec=namespace(config.get('environment','physical'))
    stages=preparation_stages(config,contract,capture_dir,physical_binding)
    output=Path(output)
    if output.exists(): raise FileExistsError('physical preparation output already exists')
    if dry_run:
        return dict(status='dry-run',mode=spec['mode'],environment=spec['environment'],stages=stages,**hashes)
    output.mkdir(parents=True)
    (output/'logs').mkdir(); (output/'.artifacts').mkdir()
    state=dict(schema_version='fake_device_preparation.v1' if spec['environment']=='fake' else 'physical_preparation.v1',
               mode=spec['mode'],environment=spec['environment'],status='running',config=config,
               started_unix_s=time.time(),stages=stages,receipts=[],execution_requires_operator_confirmation=True,**hashes)
    state['progress_estimates']=estimates(output,state)
    state['progress_estimates']['physical-target-replay']={'seconds':BASELINE['replay'],
        'basis':'历史执行与录制耗时估算；独立目标仿真待实测'}
    save(output/'run.json',state)
    deadline=time.monotonic()+contract['resources']['max_wall_time_minutes']*60
    try:
        for index,stage in enumerate(stages):
            started=time.time()
            save(output/'progress.json',dict(stage=stage['name'],status='running',started_unix_s=started,
                 next_stage=stages[index+1]['name'] if index+1<len(stages) else None,updated_unix_s=started))
            receipt=dict(stage=stage['name'],started_unix_s=started)
            try:
                argv,seconds=bounded_command(stage,deadline-time.monotonic())
                with (output/'logs'/(stage['name']+'.log')).open('xb') as log:
                    if stage['output']:
                        destination=output/'.artifacts'/stage['output']
                        destination.parent.mkdir(parents=True,exist_ok=True)
                        with destination.open('xb') as artifact:
                            completed=subprocess.run(argv,stdout=artifact,stderr=log,timeout=seconds+45)
                    elif stage['name']=='physical-target-replay':
                        completed=execute_target(stage,argv,seconds,config,output,log)
                    else:
                        completed=subprocess.run(argv,stdout=log,stderr=subprocess.STDOUT,timeout=seconds+45)
                receipt['exit_code']=completed.returncode
                if completed.returncode: raise RuntimeError(f"physical preparation stage {stage['name']} exited {completed.returncode}")
            except Exception as error:
                receipt.setdefault('exit_code',None); receipt['error']=str(error)
                raise
            finally:
                receipt.update(finished_unix_s=time.time(),actual_s=time.time()-started)
                state['receipts'].append(receipt); save(output/'run.json',state)
        started=time.time()
        save(output/'progress.json',dict(stage='verify-results',status='running',started_unix_s=started,updated_unix_s=started))
        receipt=dict(stage='verify-results',started_unix_s=started)
        try:
            values={name:json.loads((output/'.artifacts'/name).read_text()) for name in ('source.json','plan.json','hardware-binding.json')}
            validate_real_plan(values['plan.json'],values['source.json'],values['hardware-binding.json'],environment=config.get('environment','physical'))
            control=values['plan.json'].get('control',{})
            if control.get('physics_hz',0) != 30*control.get('substeps',-1):
                raise ValueError('physical preparation must freeze exactly 30 Hz commands')
            if config.get('render_replay', True):
                validate_target(output/'.artifacts/target-playback',values['source.json'],values['plan.json'],environment=spec['environment'])
                execution=json.loads((output/'.artifacts/replay-target/execution.json').read_text())
                if (execution.get('status')!='completed' or execution.get('side')!='dsw-scene'
                        or execution.get('source_run_id')!=values['source.json']['run_id']
                        or execution.get('plan_run_id')!=values['plan.json']['run_id']
                        or execution.get('timeline_digest')!=values['plan.json']['timeline_digest']
                        or execution.get('tick_count')!=len(values['plan.json']['ticks'])):
                    raise ValueError('target execution differs from prepared physical source/plan')
            receipt['exit_code']=0
        except Exception as error:
            receipt.update(exit_code=None,error=str(error)); raise
        finally:
            receipt.update(finished_unix_s=time.time(),actual_s=time.time()-started)
            state['receipts'].append(receipt); save(output/'run.json',state)
        for name in values: (output/'.artifacts'/name).replace(output/name)
        if config.get('render_replay', True):
            (output/'.artifacts/target-playback').replace(output/'target-playback')
            (output/'replay-target').mkdir(exist_ok=True)
            for name in ('execution.json','telemetry.jsonl','performance.json'):
                (output/'.artifacts/replay-target'/name).replace(output/'replay-target'/name)
        state.update(status='prepared',artifacts={name:str(output/name) for name in values},
                     target_playback=str(output/'target-playback/manifest.json') if config.get('render_replay', True) else None,
                     render_replay=config.get('render_replay', True))
    except Exception as error:
        state.update(status='failed',error=str(error))
    state['finished_unix_s']=time.time()
    save(output/'run.json',state); save(output/'final.json',state)
    save(output/'progress.json',dict(stage=state['receipts'][-1]['stage'] if state['receipts'] else 'preflight',
                                   status=state['status'],updated_unix_s=time.time()))
    return state


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','output','capture-dir','physical-binding'): parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    result=run(args.config,args.output,capture_dir=args.capture_dir,physical_binding=args.physical_binding,dry_run=args.dry_run)
    print(json.dumps(result,ensure_ascii=False))
    raise SystemExit(1 if result['status']=='failed' else 0)
