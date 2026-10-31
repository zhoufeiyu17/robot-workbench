"""Seal one physical observation, upload exact inputs, then pin this run's contract."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path, PurePosixPath
import time
from uuid import uuid4

from artifact_transport import manifest, transfer as transport_transfer, run_transport_command
from run_pipeline import absolute, below,read_contract_bytes,write_pipeline_config
from real_contract import namespace
from robot_workbench.sample_clock import clock_spec, owner_identity


def validate_config(config):
    pipeline = json.loads(Path(config['pipeline_config']).read_text())
    contract = json.loads(read_contract_bytes(pipeline))
    acquisition = config['acquisition']
    environment = namespace(config.get('environment', pipeline.get('environment', 'physical')))['environment']
    if pipeline.get('environment', 'physical') != environment:
        raise ValueError('acquisition and pipeline device environments differ')
    for name in ('profile_path', 'acquisition_binding_path'):
        if not Path(acquisition[name]).is_file():
            raise ValueError('missing physical acquisition input: ' + name)
        if environment == 'fake' and '/calibration/hardware/active' in str(Path(acquisition[name]).resolve()):
            raise ValueError('fake acquisition must use its own calibration installation snapshot')
    transport = acquisition['transport']
    root = absolute(transport['remote_root'])
    fs = contract['filesystem']
    if (below(root, pipeline['remote_run_root']) or
            not all(any(below(root, allowed) for allowed in fs.get(key, []))
                    for key in ('writable_roots', 'readonly_roots'))):
        raise ValueError('capture upload root must already be writable and declared as sealed readonly input outside the run')
    if 'endpoint' not in transport:
        raise ValueError('explicit acquisition transport endpoint required')
    return pipeline, contract


def prepare_inputs(config, output, *, read_snapshot, read_owner=None, transfer=transport_transfer, cancel=None, progress=None):
    """Injected callback is the hardware boundary; never acquire/control here."""
    preparation_started=time.monotonic()
    pipeline, contract = validate_config(config)
    acquisition = config['acquisition']
    environment = pipeline.get('environment', 'physical')
    profile = json.loads(Path(acquisition['profile_path']).read_text())
    binding = json.loads(Path(acquisition['acquisition_binding_path']).read_text())
    binding.pop('initial_state', None)  # Never reuse a previous observation's measured start.
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    name = 'physical-capture-' + uuid4().hex
    capture_dir = output / name
    from real_observation import capture, read_initial_sample, seal_initial_state, PHYSICAL_STREAM_TIMEOUT_S
    from real_pipeline import validate_capture
    measured_start = acquisition.get('measured_initial_state', False)
    if environment=='fake' and measured_start is not True:
        raise ValueError('fake-device acquisition requires measured initial-state sealing')
    if type(measured_start) is not bool or measured_start and read_owner is None:
        raise ValueError('measured initial state requires explicit boolean and read-only owner callback')
    before = read_initial_sample(read_owner,environment=environment) if measured_start else None
    if environment=='fake':
        binding['clock']=clock_spec('fake',clock_domain=before[1]['clock_domain'],source_clock_id=before[1]['source_clock_id'])
        binding['capture_owner']=owner_identity(before[1],pid_key='pid')
        from playground_pipeline import resident_wait_budget
        budget=resident_wait_budget(pipeline,contract)  # Same frozen budget and cleanup reserve as execution.
        capture_deadline=time.time_ns()+int(budget*1e9)
        if pipeline['resident_deadline_wall_ns'] is not None:
            capture_deadline=min(pipeline['resident_deadline_wall_ns']-45_000_000_000,capture_deadline)
    detector = StreamingDetection(pipeline, acquisition, output, transfer=transfer, cancel=cancel,
        progress=progress) if pipeline.get('detector_service') else None
    physical_capture={}
    if environment=='physical' and detector:
        # More opportunity to obtain the same 3 valid source seconds; no change
        # to detection, jitter/reset, cadence, or frame acceptance thresholds.
        budget=PHYSICAL_STREAM_TIMEOUT_S
        if 'max_wall_time_minutes' in contract.get('resources',{}):
            budget=min(budget,contract['resources']['max_wall_time_minutes']*60-
                       (time.monotonic()-preparation_started))
        capture_deadline=time.time_ns()+int(max(0.,budget)*1e9)
        resident_deadline=pipeline.get('resident_deadline_wall_ns')
        if resident_deadline is not None:
            capture_deadline=min(capture_deadline,resident_deadline-45_000_000_000)
        physical_capture=dict(timeout_s=PHYSICAL_STREAM_TIMEOUT_S,wall_deadline_ns=capture_deadline)
    sealed = capture(read_snapshot, profile, binding, capture_dir,
                     **({'detect_frame':detector.detect, 'progress':progress} if detector else {}),
                     **({'cancel':cancel} if cancel is not None else {}),
                     **({'environment':environment,'wall_deadline_ns':capture_deadline}
                        if environment != 'physical' else physical_capture))
    if measured_start:
        sealed['initial_state'] = seal_initial_state(before, read_initial_sample(read_owner,environment=environment,
            **({'clock':binding['clock']} if environment=='fake' else {})), profile)
        (capture_dir / 'acquisition-binding.json').write_text(json.dumps(sealed, indent=2, allow_nan=False))
    if detector:
        sealed = detector.seal(capture_dir, sealed)
    frames = validate_capture(capture_dir, sealed,environment=environment)['frames']
    names = ['profile.json', 'rgbd-manifest.json', 'acquisition-binding.json',
             *(frame['path'] for frame in frames),
             *(['detection/summary.json','detection/observations.jsonl'] if detector else [])]
    expected = manifest(capture_dir, names)
    transport = acquisition['transport']
    remote = str(PurePosixPath(transport['remote_root']) / name)
    transfer(direction='upload', local_root=str(output), local_directory=name,
             remote_root=transport['remote_root'], remote_directory=name,
             expected=expected, endpoint=transport['endpoint'],
             **({'cancel':cancel} if cancel is not None else {}))
    # Only this new sealed input gets added. Existing roots, GPU and budgets stay exact.
    frozen = deepcopy(contract)
    pins = frozen.setdefault('provenance', {}).setdefault('input_sha256', {})
    for filename in names:
        pins[str(PurePosixPath(remote) / filename)] = hashlib.sha256((capture_dir / filename).read_bytes()).hexdigest()
    pipeline_path=write_pipeline_config(pipeline,frozen,output,config_name='acquisition-pipeline.json',
        contract_name='acquisition-contract.json',transfer=transfer,cancel=cancel)
    return dict(pipeline_config=str(pipeline_path), capture_dir=remote,
                physical_binding=remote + '/acquisition-binding.json', online_detection=bool(detector))


class StreamingDetection:
    """One serial tracking session; atomic frame uploads reuse the owned model."""
    def __init__(self, pipeline, acquisition, output, *, transfer, cancel, progress):
        self.pipeline, self.acquisition, self.output = pipeline, acquisition, Path(output)
        self.transfer, self.cancel, self.progress = transfer, cancel, progress
        self.stream_id = uuid4().hex
        self.observations, self.summaries = [], []
        self.ended = False
        self.selected_cup=(pipeline.get('scene_asset_set') or {}).get('cup') or {}
        if self.selected_cup.get('pose_refinement') is not None:
            from cup_pose_refinement import validate_spec
            validate_spec(self.selected_cup['pose_refinement'],self.selected_cup['interaction_profile_sha256'])

    def rpc(self, payload, timeout):
        from run_pipeline import transport, python_command
        service = self.pipeline['detector_service']
        payload = dict(payload, owner_token=service['owner_token'], expected_identity=service['identity'],
                       stream_id=self.stream_id)
        script = '''import json,socket,sys
sys.path.insert(0,sys.argv[1])
from isaac_detection_service import send,receive
with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as c:
 c.settimeout(float(sys.argv[3]));c.connect(sys.argv[2]);send(c,json.loads(sys.argv[4]));r=receive(c)
 if r.get('exit_code')!=0:raise RuntimeError(r.get('error','detector request failed'))
 print(json.dumps(r,allow_nan=False))
'''
        command = python_command(script, str(Path(self.pipeline['detection_script']).parent),
                                 service['socket'], str(timeout), json.dumps(payload))
        completed = run_transport_command(transport(self.pipeline, 'simulated-real', command, timeout=timeout),
                        stage='检测服务调用',
                        timeout=timeout, cancel=self.cancel, capture_output=True, text=True, check=True)
        result = json.loads(completed.stdout)
        if result.get('stream_id') != self.stream_id:
            raise ValueError('detector reply belongs to another acquisition')
        return result

    def detect(self, capture_dir, row, remaining_s):
        import shutil
        from real_observation import sha
        began = time.monotonic()
        transport = self.acquisition['transport']
        name = 'observation-' + self.stream_id + '-' + str(row['capture_sequence_id'])
        directory = self.output/name
        directory.mkdir(exist_ok=False)
        shutil.copyfile(capture_dir/row['path'], directory/row['path'])
        body = dict(frame_id='d405_base_0_color_optical_frame', depth_convention='optical_z_m',
                    camera_pose_convention='world_from_ros_optical', frames=[row])
        (directory/'rgbd-manifest.json').write_text(json.dumps(body, allow_nan=False))
        expected = manifest(directory, ['rgbd-manifest.json', row['path']])
        self.transfer(direction='upload', local_root=str(self.output), local_directory=name,
            remote_root=transport['remote_root'], remote_directory=name, expected=expected,
            endpoint=transport['endpoint'], timeout=min(120., remaining_s), cancel=self.cancel)
        remaining_s -= time.monotonic()-began
        if remaining_s <= 0:raise TimeoutError('observation total wall budget exhausted during frame upload')
        remote = str(PurePosixPath(transport['remote_root'])/name)
        output = str(PurePosixPath(self.pipeline['detector_service']['allowed_output_root'])/name)
        result = self.rpc(dict(manifest=remote+'/rgbd-manifest.json', run_dir=output,
                               stream_begin=not self.observations, result_timeout_s=min(120.,remaining_s)),
                          min(125.,remaining_s))
        summary, observation = result['summary'], result['observation']
        if (summary.get('status') != 'DETECTION_FRAME_READY' or len(summary.get('inputs',[])) != 1 or
                summary['inputs'][0].get('sha256') != 'sha256:'+sha(directory/row['path'])):
            raise ValueError('stream detector did not validate the current RGB-D frame')
        if self.summaries and any(summary.get(k) != self.summaries[0].get(k) for k in
                ('tracking_session_id','runtime_config_id','node_config_sha256','adapter_sha256','service')):
            raise ValueError('detector/model identity changed inside acquisition')
        if self.selected_cup.get('pose_refinement') is not None:
            from cup_pose_refinement import refine_observation
            observation=refine_observation(observation,directory/row['path'],self.selected_cup)
        self.observations.append(observation)
        self.summaries.append(summary)
        # Preserve failed-window observations and exact detector output locations
        # even when the overall acquisition later times out or is cancelled.
        with (self.output/'observation-detections.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(observation=observation, detector_output=output,
                manifest_sha256=summary['manifest_sha256'],stream_id=self.stream_id),allow_nan=False)+'\n')
        return observation

    def seal(self, capture_dir, binding):
        from real_observation import sha
        from robot_workbench.observation_window import ContinuousObservationWindow, CUP_SPAN_M
        window = ContinuousObservationWindow()
        for observation in self.observations:window.add(observation)
        status = window.status()
        if not status['ready']:raise ValueError('cannot seal an incomplete stable observation window')
        ended = self.rpc(dict(stream_end=True), 15)
        if ended.get('cleanup_complete') is not True:raise ValueError('detector tracking cleanup incomplete')
        self.ended = True
        manifest_value = json.loads((capture_dir/'rgbd-manifest.json').read_text())
        if len(manifest_value['frames']) != len(self.observations):
            raise ValueError('sealed input/observation counts differ')
        valid = [o['source_stamp_ns'] for o in self.observations if o['pair_valid']]
        summary = dict(self.summaries[-1], status='DETECTION_WINDOW_READY', cleanup_complete=True,
            manifest_sha256='sha256:'+sha(capture_dir/'rgbd-manifest.json'),
            inputs=[dict(row,sha256='sha256:'+row['sha256']) for row in manifest_value['frames']],
            valid_pair_frames=len(valid), valid_pair_span_s=(valid[-1]-valid[0])/1e9 if len(valid)>1 else 0.,
            continuous_valid_pair_span_s=status['span_s'], continuous_valid_pair_stamps_ns=status['valid_stamps_ns'],
            cup_motion_threshold_m=CUP_SPAN_M, observation_window=status, reobserve_events=window.events,
            window_semantics='terminal valid captured samples; >=3 source seconds, >=9 frames, <=1s gaps, cup pairwise span <6mm',
            cleanup_boundary='acquisition tracking reset; shared models retained',
            stream_frame_summaries=[s['manifest_sha256'] for s in self.summaries])
        if self.selected_cup.get('pose_refinement') is not None:
            from cup_pose_refinement import source_sha256
            summary['pose_refinement']=dict(spec=self.selected_cup['pose_refinement'],source_sha256=source_sha256(),
                applies_before='unchanged full3D continuous observation window',
                raw_producer='FoundationPose; original outputs retained in each frame provenance')
        detected = capture_dir/'detection';detected.mkdir()
        (detected/'summary.json').write_text(json.dumps(summary,allow_nan=False)+'\n')
        (detected/'observations.jsonl').write_text(''.join(json.dumps(o,allow_nan=False)+'\n' for o in self.observations))
        binding['detection_files'] = dict(summary_sha256=sha(detected/'summary.json'),
                                         observations_sha256=sha(detected/'observations.jsonl'))
        (capture_dir/'acquisition-binding.json').write_text(json.dumps(binding,allow_nan=False)+'\n')
        return binding
