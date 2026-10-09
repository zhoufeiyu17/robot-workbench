#!/usr/bin/env python3
"""Bounded dual-DSW smoke pipeline; consumes deployed code and a frozen local JSON contract."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import time

from integration import DualGpuPipeline, PIPELINE_LANES
from runtime_progress import estimates
from live_preview import PREVIEW_CAMERAS, camera_frame_path, receive_stream
from playback import FILES as PLAYBACK_FILES, validate as validate_playback


ENDPOINTS = {"simulated-real": 739, "dsw-scene": 741}
PATH_KEYS = ("native_code", "workbench_code", "assets", "deps", "initial_suite", "finger_envelope",
             "detection_python", "detection_script", "detection_node_config", "detection_cuda_home")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def absolute(value):
    if not isinstance(value, str) or not value.startswith("/") or "\x00" in value or ".." in PurePosixPath(value).parts:
        raise ValueError("config paths must be absolute without parent traversal")
    return str(PurePosixPath(value))


def below(path, root):
    return PurePosixPath(path) == PurePosixPath(root) or PurePosixPath(root) in PurePosixPath(path).parents


def load_config(path):
    data = path.read_bytes()
    config = json.loads(data)
    allowed = {*PATH_KEYS, "contract_path", "finger_envelope_sha256", "prompt_file", "opening_mm",
               "remote_run_root", "record_raw", "render_replay", "motion_profile", "force_diagnostic", "diagnostic_input_root", "detector_service",
               "planning_backend", "curobo_root", "observation_input_root", "detection_input_root", "planned_input_root", "replay_profile_stop_after_ticks", "cup_contact_preset", "simulation_speed_scale", "planner_service"}
    if not isinstance(config, dict) or set(config) - allowed:
        raise ValueError("unknown pipeline configuration field")
    for key in (*PATH_KEYS, "remote_run_root", "contract_path"):
        config[key] = absolute(config.get(key))
    speed=config.get('simulation_speed_scale',1.)
    if type(speed) not in (int,float) or not math.isfinite(speed) or not .1<=speed<=1.:
        raise ValueError('simulation_speed_scale must be within0.1..1.0')
    config['simulation_speed_scale']=float(speed)
    planning_backend = config.get('planning_backend', 'native')
    if config.get('cup_contact_preset') is not None:
        if planning_backend!='motiongen' or config.get('force_diagnostic'):
            raise ValueError('cup preset requires guarded MotionGen simulation replay')
        from cup_presets import bind_preset_identity
        bind_preset_identity({},config['cup_contact_preset'])
    if planning_backend not in ('native', 'curobo', 'motiongen'):
        raise ValueError('planning_backend must be native, curobo or motiongen')
    if config.get('curobo_root') is not None:
        config['curobo_root'] = absolute(config['curobo_root'])
    if planning_backend in ('curobo', 'motiongen') and not config.get('curobo_root'):
        raise ValueError('curobo planning_backend requires curobo_root')
    root = config["remote_run_root"]
    if not below(root, "/mnt/data/feiyu.zhou/runs") or len(PurePosixPath(root).parts) < 7:
        raise ValueError("remote output must be a unique personal project/run directory")
    if type(config.get("render_replay", True)) is not bool:
        raise ValueError("render_replay must be boolean")
    if type(config.get("record_raw", False)) is not bool:
        raise ValueError("record_raw must be boolean")
    if config.get("motion_profile", "continuous-baseline") not in {"continuous-baseline", "continuous-target"}:
        raise ValueError("motion_profile must be continuous-baseline or continuous-target")
    if type(config.get("force_diagnostic", False)) is not bool:
        raise ValueError("force_diagnostic must be boolean")
    if config.get("force_diagnostic"):
        if config.get("record_raw"):
            raise ValueError("forced diagnostic replay cannot record raw training data")
        config["diagnostic_input_root"] = absolute(config.get("diagnostic_input_root", ""))
    if config.get("prompt_file") is not None:
        config["prompt_file"] = absolute(config["prompt_file"])
    if config.get("record_raw") and not config.get("prompt_file"):
        raise ValueError("raw capture requires prompt_file")
    opening = config.get("opening_mm")
    if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 < opening <= 88:
        raise ValueError("opening_mm must be finite and within (0, 88]")
    if not re.fullmatch("[0-9a-f]{64}", config.get("finger_envelope_sha256", "")):
        raise ValueError("finger envelope SHA256 is required")
    contract_bytes = Path(config["contract_path"]).read_bytes()
    contract = json.loads(contract_bytes)
    fs, resources = contract["filesystem"], contract["resources"]
    lanes = {v["instance_id"]: [v["gpu"]] for v in PIPELINE_LANES.values()}
    if (contract.get("schema_version") != "anyverse.dsw-codex-experiment.v1"
            or resources.get("lanes") != lanes or resources.get("max_gpu_count_per_instance") != 1
            or resources.get("require_gpu_idle_check") is not True
            or fs.get("allow_overwrite") is not False or fs.get("allow_delete") is not False
            or root not in fs.get("writable_roots", [])):
        raise ValueError("contract must freeze exact lane GPUs, idle check and the non-overwriting output root")
    budget = resources.get("max_wall_time_minutes")
    if type(budget) not in (int, float) or not math.isfinite(budget) or not 0 < budget <= 120:
        raise ValueError("contract needs a finite wall-time budget up to 120 minutes")
    service_lifetime=resources.get('service_lifetime_seconds')
    if service_lifetime is not None and (type(service_lifetime) is not int or not 0<service_lifetime<=172800):
        raise ValueError('service lifetime must be a positive integer up to48hours; each job retains its own wall-time budget')
    planning_budget = resources.get("max_planning_time_seconds")
    if "max_planning_time_seconds" in resources and (type(planning_budget) is not int or not 0 < planning_budget <= budget * 60 - 75):
        raise ValueError("max_planning_time_seconds must fit the total budget with shutdown reserve")
    replay_budget = resources.get("max_replay_time_seconds")
    if "max_replay_time_seconds" in resources and (type(replay_budget) is not int or not 0 < replay_budget <= budget * 60 - 45):
        raise ValueError("max_replay_time_seconds must be a positive integer within the total budget minus shutdown reserve")
    if type(resources.get("max_cpu_threads")) is not int or resources["max_cpu_threads"] < 4:
        raise ValueError("native pipeline requires a contract CPU budget of at least four threads")
    read_roots = [absolute(p) for p in fs.get("readonly_roots", [])]
    if config.get('detection_input_root') and not config.get('observation_input_root'):
        raise ValueError('completed detection requires its frozen observation input')
    for key in ('observation_input_root','detection_input_root','planned_input_root'):
        if config.get(key) is None:continue
        capture = absolute(config[key])
        if (config.get('force_diagnostic') or below(capture, root)
                or not any(below(capture, p) for p in read_roots)):
            raise ValueError('completed observation must be a declared read-only input outside this run')
        config[key] = capture
    if config.get('planned_input_root') and planning_backend!='motiongen':
        raise ValueError('accepted plan reuse is currently MotionGen-only')
    limit=config.get('replay_profile_stop_after_ticks')
    if limit is not None and (type(limit) is not int or not 1<=limit<=60 or not config.get('record_raw')):
        raise ValueError('short profiling requires 1..60 raw simulation ticks')
    for key in (*PATH_KEYS, "prompt_file", "curobo_root"):
        value = config.get(key)
        if value and (below(value, root) or not any(below(value, p) for p in read_roots)):
            raise ValueError(f"{key} must be an existing contract-declared read-only input outside this run")
    if config.get("force_diagnostic") and not any(below(config["diagnostic_input_root"], p) for p in read_roots):
        raise ValueError("diagnostic input must be a contract-declared read-only root")
    for mapping in pinned_files(contract):
        for filename, sha in mapping.items():
            filename = absolute(filename)
            if not any(below(filename, p) for p in read_roots) or not isinstance(sha, str) or not re.fullmatch("[0-9a-f]{64}", sha):
                raise ValueError("contract file pins require an allowed read-only path and SHA256")
    from detector_service import validate_config
    validate_config(config, contract)
    from planner_service import validate_config as validate_planner
    validate_planner(config, contract)
    return config, contract, {"config_sha256": digest(data), "contract_sha256": digest(contract_bytes)}


def pinned_files(contract):
    provenance = contract.get("provenance", {})
    if not isinstance(provenance, dict):
        raise ValueError("contract provenance must be an object")
    result = [provenance.get(key, {}) for key in ("input_sha256", "code_sha256", "prompt_sha256")]
    if any(not isinstance(item, dict) for item in result):
        raise ValueError("contract file pins must be path-to-SHA256 objects")
    return result


def verify_observation(config, contract):
    """Reuse only a completed, hash-pinned observation with the same scene inputs."""
    capture = Path(config.get('observation_input_root') or config['remote_run_root']+'/capture')
    if (capture/'failure.json').exists() or json.loads((capture/'final.json').read_text()).get('status') != 'observed':
        raise ValueError('observation failed or incomplete')
    manifest = json.loads((capture/'rgbd-manifest.json').read_text())
    frames = [capture/row['path'] for row in manifest['frames']]
    if len(frames)<6 or any(not below(absolute(str(p)),str(capture)) or not p.is_file() for p in frames):
        raise ValueError('incomplete RGB-D window or invalid frame path')
    if config.get('observation_input_root'):
        previous_path = capture.parent/'pipeline-config.json'
        previous = json.loads(previous_path.read_text())
        if any(previous[k]!=config[k] for k in ('native_code','assets','opening_mm','finger_envelope_sha256')):
            raise ValueError('completed observation scene inputs differ')
        previous_suite = Path(previous['initial_suite'])
        if previous_suite.read_bytes()!=Path(config['initial_suite']).read_bytes():
            raise ValueError('completed observation initial state differs')
        required = [previous_path,previous_suite,capture/'final.json',capture/'profile.json',capture/'reset.json',capture/'rgbd-manifest.json',*frames]
        pins = {k:v for mapping in pinned_files(contract) for k,v in mapping.items()}
        if any(pins.get(str(p))!=digest(p.read_bytes()) for p in required):
            raise ValueError('completed observation input hash missing or changed')
    return {'observation_verified':True,'capture_root':str(capture),'reused':bool(config.get('observation_input_root'))}


def verify_detection(config, contract):
    root = Path(config['detection_input_root'])
    pins = {k:v for mapping in pinned_files(contract) for k,v in mapping.items()}
    if any(pins.get(str(root/name))!=digest((root/name).read_bytes()) for name in ('summary.json','observations.jsonl')):
        raise ValueError('completed detection input hash missing or changed')
    summary = json.loads((root/'summary.json').read_text())
    if (summary.get('manifest_sha256')!='sha256:'+digest((Path(config['observation_input_root'])/'rgbd-manifest.json').read_bytes())
            or summary.get('adapter_sha256')!='sha256:'+pins.get(config['detection_script'],'')
            or summary.get('node_config_sha256')!='sha256:'+pins.get(config['detection_node_config'],'')):
        raise ValueError('completed detection input or adapter identity differs')
    return {'detection_verified':True,'detection_root':str(root),'reused':True}


def install_accepted_plan(config, contract):
    """Copy sealed commands and their scene, preserving the original plan identity."""
    import shutil
    previous=Path(config['planned_input_root']);root=Path(config['remote_run_root'])
    old=json.loads((previous/'pipeline-config.json').read_text())
    if any(old[k]!=config[k] for k in ('native_code','assets','opening_mm','finger_envelope_sha256')):
        raise ValueError('accepted plan scene inputs differ')
    required=[previous/'pipeline-config.json',Path(old['initial_suite']),previous/'plan.json',
              previous/'detection-bridge/source.json',previous/'detection-bridge/suite.json']
    pins={k:v for mapping in pinned_files(contract) for k,v in mapping.items()}
    if any(pins.get(str(p))!=digest(p.read_bytes()) for p in required):
        raise ValueError('accepted plan input hash missing or changed')
    if Path(old['initial_suite']).read_bytes()!=Path(config['initial_suite']).read_bytes():
        raise ValueError('accepted plan original reset state differs')
    source=json.loads(required[3].read_text());plan=json.loads(required[2].read_text())
    if source.get('synthetic_fixture') or source.get('test_fixture'):raise ValueError('fixture plan cannot resume formal replay')
    pipeline=DualGpuPipeline();pipeline.ingest_source(source);pipeline.accept_plan(plan)
    bridge=root/'detection-bridge';bridge.mkdir()
    for p in required[3:]:shutil.copyfile(p,bridge/p.name)
    shutil.copyfile(required[2],root/'plan.json')
    return dict(accepted_plan_reused=True,timeline_digest=plan['timeline_digest'],parent_run=str(previous))


def ssh(side, command):
    args = ["ssh", "-i", str(Path.home() / ".ssh/id_ed25519_dsw_codex"), "-p", str(ENDPOINTS[side]),
            "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
            "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3"]
    args += ["-o", "HostKeyAlias=[47.116.111.83]:" + str(ENDPOINTS[side])]
    return args + ["root@47.116.111.83", command]


def python_command(script, *args):
    values = list(map(str, args))
    command = shlex.join(["python3", "-I", "-B", "-c", script, *values])
    if len(command.encode()) < 32768:
        return command
    # Large SHA manifests exceed Linux's per-argument limit; decode inside Python.
    import base64
    import zlib
    payload = base64.b64encode(zlib.compress(json.dumps([script, values]).encode())).decode()
    bootstrap = ("import base64,json,sys,zlib; "
                 "s,a=json.loads(zlib.decompress(base64.b64decode(sys.argv[1]))); "
                 "sys.argv=['-c',*a]; exec(compile(s,'<remote>','exec'))")
    return shlex.join(["python3", "-I", "-B", "-c", bootstrap, payload])


def pull_telemetry(side, remote_path, local_path):
    """Best-effort read-only tail source; replay may still be writing the JSONL file."""
    script = ("import pathlib,sys; p=pathlib.Path(sys.argv[1]); "
              "sys.stdout.buffer.write(p.read_bytes()) if p.is_file() else sys.exit(2)")
    temp = Path(str(local_path) + ".partial")
    try:
        with temp.open("wb") as stream:
            result = subprocess.run(ssh(side, python_command(script, remote_path)), stdout=stream,
                                    stderr=subprocess.DEVNULL, timeout=12)
        if result.returncode == 0:
            temp.replace(local_path)
        else:
            temp.unlink(missing_ok=True)
    except (OSError, subprocess.SubprocessError):
        temp.unlink(missing_ok=True)


def replay_receipt(config, side, label, output):
    """A zero SSH exit is not an Isaac completion receipt."""
    script = """import json,pathlib,sys
# workbench_replay_receipt
root=pathlib.Path(sys.argv[1])
result={}
for name in ('failure', 'final', 'execution'):
 p=root/(name+'.json')
 if p.is_file():
  value=json.loads(p.read_text())
  result[name]={k:value[k] for k in ('status','error','run_id','ticks','tick_count','timeline_digest') if k in value}
print(json.dumps(result))
"""
    command = python_command(script, config["remote_run_root"] + "/replay-" + label)
    result = subprocess.run(ssh(side, "bash -c " + shlex.quote(command)), capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise RuntimeError("cannot read replay completion receipt: " + result.stderr[-2000:])
    receipt = json.loads(result.stdout)
    (output / ("replay-" + label)).mkdir(exist_ok=True)
    save(output / ("replay-" + label) / "completion-receipt.json", receipt)
    if receipt.get("failure"):
        raise RuntimeError("native execution failed: " + receipt["failure"].get("error", "unknown native failure"))
    diagnostic = config.get("force_diagnostic", False)
    final, execution = receipt.get("final", {}), receipt.get("execution", {})
    if (final.get("status") != ("diagnostic-complete" if diagnostic else "replay-complete")
            or execution.get("status") != ("diagnostic-complete" if diagnostic else "completed")
            or not execution.get("timeline_digest") or not execution.get("tick_count")
            or final.get("ticks") != execution.get("tick_count")):
        raise RuntimeError("missing or incomplete native replay completion receipt: " + json.dumps(receipt))
    return receipt


def start_preview_stream(config, side, label, local_path, camera="base_0"):
    """Keep one bounded SSH stream for the latest remote camera frame."""
    remote = str(camera_frame_path(config["remote_run_root"] + "/replay-" + label, camera))
    local_path.parent.mkdir(parents=True, exist_ok=True)
    workbench = config["workbench_code"]
    command = "cd " + shlex.quote(config["remote_run_root"]) + " && " + shlex.join([
        "env", "PYTHONPATH=" + workbench, "python3", "-B", workbench + "/live_preview.py", "stream",
        "--path", remote, "--hz", "30"])
    process = subprocess.Popen(ssh(side, "bash -c " + shlex.quote(command)),
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    return process, process.stdout


def stop_preview_stream(process, stream):
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
    if stream is not None:
        stream.close()


GPU_CHECK = """import json, subprocess, sys
gpu=sys.argv[1]
line=subprocess.check_output(['nvidia-smi','-i',gpu,'--query-gpu=uuid,memory.used,utilization.gpu','--format=csv,noheader,nounits'],text=True).strip()
uuid,memory,util=[s.strip() for s in line.split(',')]
processes=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid','--format=csv,noheader,nounits'],text=True)
assert not any(row.split(',')[0].strip()==uuid for row in processes.splitlines()), 'selected GPU has a compute process'
assert int(memory)<=128 and int(util)<=5, 'selected GPU is not idle'
print(json.dumps({'gpu':int(gpu),'uuid':uuid,'memory_mib':int(memory),'utilization':int(util)}))
"""

AFFINITY_EXEC = ("import os,sys; cores=sorted(os.sched_getaffinity(0))[:4]; "
                 "assert cores, 'empty CPU affinity'; os.sched_setaffinity(0,cores); "
                 "os.execvp(sys.argv[1],sys.argv[1:])")


def isaac_environment(c,side):
    """Share native GPU acceptance dependencies across both workbench modes."""
    settings={"CUDA_VISIBLE_DEVICES": str(PIPELINE_LANES[side]["gpu"]), "PYTHONNOUSERSITE": "1",
        "PYTHONDONTWRITEBYTECODE": "1", "OMP_NUM_THREADS": "4", "MKL_NUM_THREADS": "4", "MAX_JOBS": "4",
        "ROBOT_SIM_ASSET_ROOT": c["assets"],
        "PYTHONPATH": f"{c['deps']}:/workspace/isaaclab/source/isaaclab:{c['native_code']}/source/robot_sim:{c['workbench_code']}"}
    if c.get('planning_backend')=='motiongen':
        install=c['curobo_root'];cache=c['remote_run_root']+'/cache/runtime-'+side
        settings.update(PYTHONPATH=install+'/python:'+install+'/deps:'+settings['PYTHONPATH'],
            CUDA_HOME=install+'/cuda-overlay',TORCH_EXTENSIONS_DIR=cache+'/torch',WARP_CACHE_PATH=cache+'/warp',XDG_CACHE_HOME=cache)
    return settings


def build_plan(config, contract):
    from detector_service import gpu_check_command
    c = config
    root, wb, native = (c[k] for k in ("remote_run_root", "workbench_code", "native_code"))
    run_id = PurePosixPath(root).name
    capture, detected = c.get('observation_input_root') or root + "/capture", root + "/detection-bridge"
    stages = []

    def stage(name, side, command, output=None, group=None):
        timeout_s = contract["resources"].get("max_replay_time_seconds", 600) if group == "replay" else 600
        if name in {"geometry", "place", "motiongen-scene", "motiongen-plan"}:
            timeout_s = contract["resources"].get("max_planning_time_seconds", 570) + 30
        wall_budget=contract['resources'].get('stage_wall_seconds',{}).get(name)
        if wall_budget is not None:
            if type(wall_budget) is not int or wall_budget<=45:raise ValueError('stage wall budget must reserve shutdown time')
            timeout_s=min(timeout_s,wall_budget-45)
        stages.append({"name": name, "side": side, "timeout_s": timeout_s,"wall_budget_s":wall_budget,
                       "argv": ssh(side, f"timeout -k 30 {timeout_s} " +
                       shlex.join(["bash", "-c" if output else "-lc", command])), "output": output, "group": group})

    def env(side):
        return isaac_environment(c,side)

    def command(side, args, overrides=None):
        settings = {**env(side), **(overrides or {})}
        return "cd " + shlex.quote(root) + " && " + shlex.join([
            "env", *[f"{k}={v}" for k, v in settings.items()], "python3", "-B", "-c", AFFINITY_EXEC, *args])

    def gpu_guard(side):
        guard=gpu_check_command(c,side)
        expected=contract['resources'].get('expected_gpu_uuids',{}).get(PIPELINE_LANES[side]['instance_id'])
        if expected:
            check="import subprocess,sys; assert subprocess.check_output(['nvidia-smi','--query-gpu=uuid','--format=csv,noheader'],text=True).strip()==sys.argv[1], 'GPU identity changed'"
            guard+=' && '+python_command(check,expected)
        return guard

    def gpu_stage(name, side, args, overrides=None):
        limits=contract['resources']
        if name in {"geometry", "place", "motiongen-scene", "motiongen-plan"}:
            owned_budget=limits.get('max_planning_time_seconds',570)
            wall_budget=limits.get('stage_wall_seconds',{}).get(name)
            if wall_budget is not None:owned_budget=min(owned_budget,wall_budget-50)
        else:
            wall_budget=limits.get('stage_wall_seconds',{}).get(name,limits.get('max_replay_time_seconds',1800) if name.startswith('replay-') else 600)
            owned_budget=max(1,wall_budget-50)
        args = ["python3", "-B", wb + "/owned_process.py", "--timeout",str(owned_budget),"--",*args]
        stage(name, side, gpu_guard(side) + " && " + command(side, args, overrides),
                       group="replay" if "replay-" in name else None)
        stages[-1]['gpu_work']=True

    check = """import hashlib,json,pathlib,shutil,subprocess,sys
c=json.loads(sys.argv[1]); root=pathlib.Path(c['remote_run_root'])
assert not root.exists(), 'output already exists'
for key in json.loads(sys.argv[2]):
 p=pathlib.Path(c[key]); assert p.exists(), 'missing input '+key
for p in [pathlib.Path(c['native_code'])/'scripts/run_coffee_episodes.py',
          *[pathlib.Path(c['workbench_code'])/name for name in ('isaac_lane.py','source_from_detection.py','export_plan.py','integration.py','contract.py')],
          pathlib.Path('/isaac-sim/python.sh')]:
 assert p.is_file(), 'missing deployed entrypoint '+str(p)
if c.get('planning_backend')=='motiongen':
 for p in [pathlib.Path(c['workbench_code'])/'motiongen_plan.py',pathlib.Path(c['curobo_root'])/'python/curobo/__init__.py']:
  assert p.is_file(), 'missing MotionGen entrypoint/runtime '+str(p)
for pins in json.loads(sys.argv[3]):
 for filename,expected in pins.items():
  digest=hashlib.sha256()
  with pathlib.Path(filename).open('rb') as stream:
   for block in iter(lambda: stream.read(1024*1024),b''): digest.update(block)
  assert digest.hexdigest()==expected, 'contract file SHA mismatch '+filename
assert hashlib.sha256(pathlib.Path(c['finger_envelope']).read_bytes()).hexdigest()==c['finger_envelope_sha256'], 'finger envelope SHA mismatch'
suite=json.loads(pathlib.Path(c['initial_suite']).read_text()); assert len(suite['episodes'])==1, 'one episode required'
if c.get('record_raw'):
 assert (pathlib.Path(c['workbench_code'])/'playback.py').is_file(), 'missing playback exporter'
 prompt=json.loads(pathlib.Path(c['prompt_file']).read_text())
 assert isinstance(prompt,dict) and all(isinstance(prompt.get(k),str) and prompt[k].strip() for k in ('text','language','version')), 'invalid raw prompt'
 encoders=subprocess.check_output(['ffmpeg','-hide_banner','-encoders'],text=True,stderr=subprocess.STDOUT)
 assert 'libx265' in encoders, 'raw capture requires CPU libx265'
 assert 'libx264' in encoders and shutil.which('ffprobe'), 'browser playback requires libx264 and ffprobe'
assert shutil.disk_usage(root.parent).free > 10*1024**3, 'less than 10 GiB free'
print(json.dumps({'inputs_verified':True,'output_absent':True}))
"""
    for side in ENDPOINTS:
        inputs = [key for key in PATH_KEYS if side == "simulated-real" or not key.startswith("detection_")]
        inputs += ["prompt_file"] if c.get("record_raw") else []
        stage("preflight-" + side, side, gpu_guard(side) + " && " +
              python_command(check, json.dumps(c), json.dumps(inputs), json.dumps(pinned_files(contract))))
    init = """import json,pathlib,sys
c=json.loads(sys.argv[1]); root=pathlib.Path(c['remote_run_root']); root.mkdir()
(root/'barrier').mkdir(); (root/'cache').mkdir()
(root/'contract.json').write_text(sys.argv[2]); (root/'pipeline-config.json').write_text(sys.argv[1])
"""
    if c.get("force_diagnostic"):
        copy = """import json,pathlib,shutil,sys
c=json.loads(sys.argv[1]); root=pathlib.Path(c['remote_run_root']); source=pathlib.Path(c['diagnostic_input_root'])
for name in ('plan.json',): shutil.copy2(source/name, root/name)
shutil.copytree(source/'detection-bridge', root/'detection-bridge')
assert (root/'plan.json').is_file() and (root/'detection-bridge/source.json').is_file()
"""
        stage("initialize", "simulated-real", python_command(init, json.dumps(c), json.dumps(contract)))
        stage("diagnostic-input", "simulated-real", python_command(copy, json.dumps(c)))
    else:
        stage("initialize", "simulated-real", python_command(init, json.dumps(c), json.dumps(contract)))
    stage("shared-storage", "dsw-scene", python_command(
        "import json,pathlib,sys; assert json.loads(pathlib.Path(sys.argv[1]).read_text())==json.loads(sys.argv[2])",
        root + "/pipeline-config.json", json.dumps(c)))
    if c.get('planned_input_root'):
        stage('resume-plan','dsw-scene',python_command(
            'import json,sys; sys.path.insert(0,sys.argv[1]); from run_pipeline import install_accepted_plan; print(json.dumps(install_accepted_plan(json.loads(sys.argv[2]),json.loads(sys.argv[3]))))',
            wb,json.dumps(c),json.dumps(contract)))
    isaac = ["/isaac-sim/python.sh", "-u", "-B"]
    if c.get("force_diagnostic"):
        for side, label in (("simulated-real", "source"), ("dsw-scene", "target")):
            suite = c["initial_suite"] if label == "source" else root + "/detection-bridge/suite.json"
            args = [*isaac, wb + "/isaac_lane.py", "--mode", "replay", "--gpu", str(PIPELINE_LANES[side]["gpu"]),
                    "--instance", PIPELINE_LANES[side]["instance_id"], "--side", side, "--run-id", run_id + "-" + label,
                    "--suite", suite, "--assets", c["assets"], "--output", root + "/replay-" + label,
                    "--plan", root + "/plan.json", "--source", root + "/detection-bridge/source.json",
                    "--barrier", root + "/barrier", "--force-diagnostic-replay"]
            args += ["--whitebox-fast"]
            gpu_stage("diagnostic-replay-" + label, side, args)
        for local, remote in {"performance-source.json": "replay-source/performance.json",
                              "performance-target.json": "replay-target/performance.json"}.items():
            stage("fetch-" + local, "simulated-real", python_command(
                "import pathlib,sys; p=pathlib.Path(sys.argv[1]); assert p.stat().st_size<=32*1024**2; sys.stdout.buffer.write(p.read_bytes())",
                root + "/" + remote), output=local)
        return stages
    source_side = "simulated-real"
    if not c.get('observation_input_root'):
        gpu_stage("observe", source_side, [*isaac, wb + "/isaac_lane.py", "--mode", "observe", "--gpu", "0",
            "--instance", PIPELINE_LANES[source_side]["instance_id"], "--side", source_side, "--run-id", run_id + "-capture",
            "--suite", c["initial_suite"], "--assets", c["assets"], "--output", capture,
            "--observation-seconds", "8", "--gripper-opening-mm", str(c["opening_mm"])])
    stage("observe-verify", source_side, python_command(
        'import json,sys; sys.path.insert(0,sys.argv[1]); from run_pipeline import verify_observation; print(json.dumps(verify_observation(json.loads(sys.argv[2]),json.loads(sys.argv[3]))))',
        wb,json.dumps(c),json.dumps(contract)))
    fp_env = {"PYTHONPATH": "", "CUDA_HOME": c["detection_cuda_home"], "TORCH_EXTENSIONS_DIR": root + "/cache/torch-extensions",
              "XDG_CACHE_HOME": root + "/cache", "PATH": c["detection_cuda_home"] + "/bin:" +
              str(PurePosixPath(c["detection_python"]).parent) + ":/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"}
    detection = c.get('detection_input_root') or root+'/detection'
    if c.get('detection_input_root'):
        stage('detect-verify',source_side,python_command(
            'import json,sys; sys.path.insert(0,sys.argv[1]); from run_pipeline import verify_detection; print(json.dumps(verify_detection(json.loads(sys.argv[2]),json.loads(sys.argv[3]))))',
            wb,json.dumps(c),json.dumps(contract)))
    elif c.get("detector_service"):
        service = c["detector_service"]
        gpu_stage("detect", source_side, [c["detection_python"], "-B", c["detection_script"], "--request",
            "--socket", service["socket"], "--owner-token", service["owner_token"],
            "--expected-identity", json.dumps(service["identity"]),
            "--manifest", capture + "/rgbd-manifest.json", "--run-dir", root + "/detection"], fp_env)
    else:
        gpu_stage("detect", source_side, [c["detection_python"], "-B", c["detection_script"], "--manifest",
            capture + "/rgbd-manifest.json", "--node-config", c["detection_node_config"], "--run-dir", root + "/detection",
            "--result-timeout-s", "240"], fp_env)
    stage("bridge", source_side, command(source_side, ["python3", "-B", wb + "/source_from_detection.py",
        "--profile", capture + "/profile.json", "--detection-dir", detection, "--output", detected,
        "--run-id", run_id + "-source",*(['--simulation-initial-suite',c['initial_suite']] if c.get('planning_backend')=='motiongen' else [])]))
    if c.get('planning_backend') == 'motiongen':
        scene_dir, plan_dir = root+'/motiongen-scene', root+'/motiongen-plan'
        gpu_stage('motiongen-scene', 'dsw-scene', [*isaac, wb+'/isaac_lane.py', '--mode', 'geometry-inputs',
            '--gpu', '0', '--instance', PIPELINE_LANES['dsw-scene']['instance_id'], '--side', 'dsw-scene',
            '--run-id', run_id+'-motiongen-scene', '--suite', detected+'/suite.json', '--source', detected+'/source.json',
            '--assets', c['assets'], '--output', scene_dir, '--gripper-opening-mm', str(c['opening_mm']),
            '--finger-envelope', c['finger_envelope'], '--finger-envelope-sha256', c['finger_envelope_sha256']])
        install, cache = c['curobo_root'], root+'/cache/curobo'
        settings = dict(PYTHONPATH=install+'/python:'+install+'/deps:'+c['deps']+':'+native+'/source/robot_sim:'+wb,
            CUDA_HOME=install+'/cuda-overlay', LD_LIBRARY_PATH=install+'/cuda-overlay/lib64',
            TORCH_EXTENSIONS_DIR=cache+'/torch', WARP_CACHE_PATH=cache+'/warp', XDG_CACHE_HOME=cache,
            TORCH_CUDA_ARCH_LIST='12.0', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
        if c.get('planner_service'):
            from planner_service import private_spec_arguments
            private=private_spec_arguments(c)
            request=dict(input=scene_dir+'/planning-input.json',source=detected+'/source.json',
                output=plan_dir,speed_scale=c['simulation_speed_scale'])
            gpu_stage('motiongen-plan','dsw-scene',[*isaac,wb+'/curobo_worker.py','--request',json.dumps(request),
                '--service-spec-file',private['spec_path'],'--spec-sha256',private['spec_sha256']],settings)
        else:
            gpu_stage('motiongen-plan', 'dsw-scene', [*isaac, wb+'/motiongen_plan.py',
                '--input', scene_dir+'/planning-input.json', '--source', detected+'/source.json',
                '--native-code', native, '--assets', c['assets'], '--output', plan_dir, '--speed-scale', str(c['simulation_speed_scale'])], settings)
        publish = ('import json,pathlib,sys; from export_plan import publish_motiongen; '
                   'r=pathlib.Path(sys.argv[1]); print(json.dumps(publish_motiongen('
                   'r/"detection-bridge/source.json",r/"motiongen-plan",r/"plan.json",json.loads(sys.argv[2]))))')
        stage('motiongen-publish', 'dsw-scene', command('dsw-scene', [*isaac, '-c', publish, root,json.dumps(c.get('cup_contact_preset'))]))
    else:
        for mode in ("geometry", "place"):
            gpu_stage(mode, "dsw-scene", [*isaac, native + "/scripts/run_coffee_episodes.py", "--asset-root", c["assets"],
                "--suite", detected + "/suite.json", "--finger-envelope", c["finger_envelope"], "--finger-envelope-sha256",
                c["finger_envelope_sha256"], "--output", root + "/" + mode, "--mode", mode,
                "--motion-profile", c.get("motion_profile", "continuous-baseline"), "--gripper-opening-mm",
                str(c["opening_mm"]), "--render-gpu", str(PIPELINE_LANES["dsw-scene"]["gpu"])])
            if mode == "geometry":
                stage("geometry-verify", "dsw-scene", python_command(
                    "import json,pathlib,sys; root=pathlib.Path(sys.argv[1]); "
                    "episode=json.loads((root/'detection-bridge/suite.json').read_text())['episodes'][0]['episode_id']; "
                    "report=json.loads((root/'geometry/scene-000'/episode/'report.json').read_text()); "
                    "assert report.get('outcome')=='not_run' and report.get('diagnostic')=='complete_geometry_verified', 'geometry verification failed'",
                    root))
        export = ("import json,pathlib,sys; from export_plan import export; "
                  "root=pathlib.Path(sys.argv[1]); episode=json.loads((root/'detection-bridge/suite.json').read_text())['episodes'][0]['episode_id']; "
                  "export(root/'detection-bridge/source.json',root/'place/scene-000'/episode,root/'plan.json')")
        stage("export", "dsw-scene", command("dsw-scene", [*isaac, "-c", export, root]))
    for side, label in ((source_side, "source"), ("dsw-scene", "target")):
        # Preserve the observation's original reset configuration, including authored orientation/randomization.
        # capture/suite.json is an aggregated GT observation and is not the original physical configuration.
        suite = c["initial_suite"] if label == "source" else detected + "/suite.json"
        args = [*isaac, wb + "/isaac_lane.py", "--mode", "replay", "--gpu", str(PIPELINE_LANES[side]["gpu"]),
            "--instance", PIPELINE_LANES[side]["instance_id"], "--side", side, "--run-id", run_id + "-" + label,
            "--suite", suite, "--assets", c["assets"], "--output", root + "/replay-" + label,
            "--plan", root + "/plan.json", "--source", detected + "/source.json"]
        if c.get('planning_backend')=='motiongen':args += ['--independent-replay']
        else:args += ['--barrier',root+'/barrier']
        if c.get('replay_profile_stop_after_ticks') is not None:
            args+=['--profile-stop-after-ticks',str(c['replay_profile_stop_after_ticks'])]
        if c.get("record_raw"):
            args += ["--record-raw", "--raw-prompt-file", c["prompt_file"]]
        gpu_stage("replay-" + label, side, args,
                  {"ROBOT_SIM_PROFILE_CAPTURE": "1"} if c.get("record_raw") else None)
    artifacts = {"source.json": "detection-bridge/source.json", "plan.json": "plan.json",
                 "execution-source.json": "replay-source/execution.json", "execution-target.json": "replay-target/execution.json",
                 "source.png": "replay-source/final-camera.png", "target.png": "replay-target/final-camera.png"}
    if c.get("record_raw"):
        stage("export-playback", source_side, shlex.join([
            "env", "CUDA_VISIBLE_DEVICES=-1", "PYTHONDONTWRITEBYTECODE=1", "python3", "-B",
            wb + "/playback.py", "--run-root", root]))
        artifacts.update({"playback/" + name: "playback/" + name for name in ("manifest.json", *PLAYBACK_FILES)})
    for local, remote in artifacts.items():
        stage("fetch-" + local.replace("/", "-"), source_side, python_command(
            "import pathlib,sys; p=pathlib.Path(sys.argv[1]); assert p.stat().st_size<=32*1024**2, 'artifact too large'; sys.stdout.buffer.write(p.read_bytes())",
            root + "/" + remote), output=local)
    if c.get('planned_input_root'):
        stages=[s for s in stages if s['name'].startswith(('preflight-','fetch-'))
                or s['name'] in ('initialize','shared-storage','resume-plan','export-playback') or s['group']=='replay']
    return stages


def save(path, value):
    temp = path.with_suffix(".partial")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def verify(output, record_raw, *, run_id=None):
    pipeline = DualGpuPipeline()
    source = json.loads((output / "source.json").read_text())
    if source.get("adapter") != "foundationpose" or source.get("observation_mode") != "offline_replay":
        raise ValueError("pipeline requires actual offline FoundationPose output")
    pipeline.ingest_source(source)
    pipeline.accept_plan(json.loads((output / "plan.json").read_text()))
    for side in ("source", "target"):
        receipt = json.loads((output / ("execution-" + side + ".json")).read_text())
        if receipt.get("side") != ("simulated-real" if side == "source" else "dsw-scene"):
            raise ValueError("execution file belongs to the wrong lane")
        if not receipt.get("actual_episode_scene"):
            raise ValueError("execution must retain the actual physical scene")
        pipeline.record_execution(receipt)
        if record_raw and (receipt.get("recording", {}).get("capture_status") != "complete"
                           or receipt["recording"].get("validation") != "passed"):
            raise ValueError("requested raw capture did not seal successfully")
        if not (output / (side + ".png")).read_bytes().startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("camera artifact is not PNG")
    summary = pipeline.summary()
    if summary["execution"] not in ("replay-complete", "synchronized"):
        raise ValueError("both frozen replays must be complete")
    if record_raw:
        summary["playback"] = validate_playback(output / "playback", pipeline.plan["timeline_digest"], run_id=run_id)
    return summary


def bounded_command(stage, remaining):
    # Reserve the timeout kill grace and SSH delivery margin inside the total contract budget.
    limit = stage["timeout_s"]
    seconds = min(limit, int(remaining - 45))
    if seconds < 1:
        raise TimeoutError("contract wall-time budget exhausted before command start (45s shutdown reserve)")
    argv = list(stage["argv"])
    remote = shlex.split(argv[-1])
    if remote[:4] != ["timeout", "-k", "30", str(limit)]:
        raise ValueError("unexpected remote timeout wrapper")
    remote[3] = str(seconds)
    argv[-1] = shlex.join(remote)
    return argv, seconds


def run(config_path, output, dry_run=False):
    config, contract, hashes = load_config(config_path)
    stages = build_plan(config, contract)
    if output.exists():
        raise FileExistsError("local output already exists")
    if dry_run:
        return {"status": "dry-run", "config": config, **hashes, "stages": stages}
    output.mkdir(parents=True, exist_ok=False)
    (output / "logs").mkdir()
    artifacts = output / ".artifacts"
    artifacts.mkdir()
    begin = time.monotonic()
    deadline = begin + contract["resources"]["max_wall_time_minutes"] * 60
    window=contract.get('window')
    if window:
        if not window['start_unix']<=time.time()<window['stop_unix']<window['hard_end_unix']:
            raise ValueError('signed runtime window is not active')
        deadline=min(deadline,begin+window['hard_end_unix']-time.time())
    state = {"status": "running", "started_unix_s": time.time(), "config": config, **hashes,
             "stages": stages, "receipts": []}
    state["progress_estimates"] = estimates(output, state)
    save(output / "run.json", state)

    def execute(stage):
        log_path = output / "logs" / (stage["name"] + ".log")
        remaining = deadline - time.monotonic()
        if window and stage.get('gpu_work'):
            remaining=min(remaining,window['stop_unix']-time.time())
        started = time.time()
        launched = False
        try:
            argv, seconds = bounded_command(stage, remaining)
            local_timeout=min(seconds+45,stage.get('wall_budget_s') or seconds+45)
            with log_path.open("xb") as log:
                if stage["output"]:
                    (artifacts / stage["output"]).parent.mkdir(parents=True, exist_ok=True)
                    with (artifacts / stage["output"]).open("xb") as artifact:
                        launched = True
                        result = subprocess.run(argv, stdout=artifact, stderr=log, timeout=local_timeout)
                elif "replay-" in stage["name"]:
                    # Keep the UI fed while Isaac is stepping. The replay remains the
                    # source of truth; polling only copies its append-only telemetry.
                    side = stage["side"]
                    label = "source" if side == "simulated-real" else "target"
                    local = output / ("telemetry-" + label + ".jsonl")
                    remote = config["remote_run_root"] + "/replay-" + label + "/telemetry.jsonl"
                    previews = []
                    import threading
                    for camera in PREVIEW_CAMERAS:
                        preview_path = camera_frame_path(output / ("replay-" + label), camera)
                        preview_process = preview_stream = None
                        try:
                            preview_process, preview_stream = start_preview_stream(config, side, label, preview_path, camera)
                            reader = None
                            if preview_stream is not None:
                                reader = threading.Thread(target=receive_stream,
                                                          args=(preview_stream, preview_path), daemon=True)
                                reader.start()
                            previews.append((preview_process, preview_stream, reader))
                        except (OSError, subprocess.SubprocessError):
                            stop_preview_stream(preview_process, preview_stream)
                    launched = True
                    try:
                        process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
                        while process.poll() is None:
                            pull_telemetry(side, remote, local)
                            time.sleep(1.0)
                        pull_telemetry(side, remote, local)
                        result = subprocess.CompletedProcess(argv, process.returncode)
                        replay_receipt(config, side, label, output)
                    finally:
                        for preview_process, preview_stream, reader in previews:
                            stop_preview_stream(preview_process, preview_stream)
                            if reader is not None:
                                reader.join(timeout=5)
                else:
                    launched = True
                    result = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, timeout=local_timeout)
            return {"stage": stage["name"], "started_unix_s": started, "finished_unix_s": time.time(),
                    "exit_code": result.returncode, "log": str(log_path), "remote_timeout_s": seconds}
        except Exception as error:
            with log_path.open("ab") as log:
                log.write(("\n[pipeline] " + str(error) + "\n").encode())
            return {"stage": stage["name"], "started_unix_s": started, "finished_unix_s": time.time(), "exit_code": None, "error": str(error), "log": str(log_path),
                    "remote_cleanup": "unverified; bounded remote timeout remains in force" if launched else "not started"}

    try:
        i = 0
        while i < len(stages):
            stage = stages[i]
            save(output / "progress.json", {"stage": stage["group"] or stage["name"], "status": "running",
                                             "updated_unix_s": time.time()})
            if stage["group"] == "replay":
                with ThreadPoolExecutor(max_workers=2) as pool:
                    receipts = list(pool.map(execute, stages[i:i+2]))
                i += 2
            else:
                receipts = [execute(stage)]
                i += 1
            state["receipts"].extend(receipts)
            save(output / "run.json", state)
            if any(item["exit_code"] != 0 for item in receipts):
                raise RuntimeError("stage failed: " + "; ".join(
                    f"{item['stage']} (exit={item['exit_code']}, log={item['log']})" +
                    (": " + item["error"] if item.get("error") else "")
                    for item in receipts if item["exit_code"] != 0))
        verification_started = time.time()
        save(output / "progress.json", {"stage": "verify-results", "status": "running",
                                        "updated_unix_s": verification_started})
        if config.get("force_diagnostic"):
            for artifact in artifacts.iterdir():
                artifact.rename(output / artifact.name)
            diagnostics = {}
            for name in ("performance-source.json", "performance-target.json"):
                value = json.loads((output / name).read_text())
                diagnostics[name] = value
            final = {"status": "diagnostic-complete", "diagnostic": diagnostics, "synchronized": False,
                     "promotion": "forbidden", "timing_boundary": "diagnostic replay; no success claim",
                     "artifacts": {p.name: digest(p.read_bytes()) for p in output.iterdir()
                                   if p.suffix in (".json", ".png") and p.name not in ("run.json", "progress.json")}}
        else:
            summary = verify(artifacts, config.get("record_raw", False), run_id=PurePosixPath(config["remote_run_root"]).name)
            for artifact in artifacts.iterdir():
                artifact.rename(output / artifact.name)
            final = {"status": "replay-complete", "pipeline": summary, "synchronized": False,
                     "timing_boundary": "per-tick lockstep; no real-time synchronization claim",
                     "artifacts": {p.name: digest(p.read_bytes()) for p in output.iterdir()
                                   if p.suffix in (".json", ".png") and p.name not in ("run.json", "progress.json")}}
    except Exception as error:
        if config.get("force_diagnostic"):
            diagnostics = {}
            for label in ("source", "target"):
                path = output / ("telemetry-" + label + ".jsonl")
                rows = []
                if path.is_file():
                    for line in path.read_text().splitlines():
                        try:
                            rows.append(json.loads(line))
                        except json.JSONDecodeError:
                            continue
                diagnostics[label] = {
                    "ticks": len(rows),
                    "last_tick": rows[-1].get("tick") if rows else None,
                    "terminal_failure": next((row.get("event") for row in reversed(rows) if row.get("event")), None),
                    "collision_events": [event for row in rows for event in row.get("diagnostic_events", [])],
                }
            final = {"status": "diagnostic-complete", "error": str(error), "synchronized": False,
                     "promotion": "forbidden", "diagnostic": diagnostics,
                     "timing_boundary": "partial diagnostic replay; no success claim",
                     "remote_cleanup": "unverified; no untracked process was stopped"}
        else:
            final = {"status": "failed", "error": str(error), "synchronized": False,
                     "remote_cleanup": "unverified; no untracked process was stopped"}
    if "verification_started" in locals():
        state["receipts"].append({"stage": "verify-results", "started_unix_s": verification_started,
                                  "finished_unix_s": time.time(), "exit_code": 1 if final.get("error") else 0})
    final.update(finished_unix_s=time.time(), wall_seconds=time.monotonic() - begin)
    state["status"] = final["status"]
    save(output / "run.json", state)
    save(output / "final.json", final)
    save(output / "progress.json", {"stage": ("verify-results" if "verification_started" in locals() else stage["group"] or stage["name"]),
                                    "status": final["status"], "updated_unix_s": time.time()})
    return final


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    result = run(args.config, args.output, args.dry_run)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(1 if result["status"] == "failed" else 0)
