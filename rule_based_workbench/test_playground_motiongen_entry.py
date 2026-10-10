"""Actual Playground prepare caller; synthetic cameras/GPU only, no SSH or hardware.

Configuration, capture sealing, artifact hashes, detection/model binding and final
physical-plan validation are real. This is CPU integration evidence, not a GPU
geometry or real RGB-D qualification.
"""
from contextlib import ExitStack
from copy import deepcopy
import json
from pathlib import Path
import shlex
import shutil
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from a1_asset_validation import ROBOT, sha256, validate_assets
from control_motion import validate_control
from integration import PIPELINE_LANES, _digest
from physical_motiongen import freeze_physical, gripper_offset_mm, target_vector
import playground_pipeline as pipeline
from real_acquisition import prepare_inputs
from real_observation import capture
from real_pipeline import bind_detection, import_capture, preparation_stages
from run_pipeline import PATH_KEYS, load_config
from source_from_detection import convert
from test_detection_bridge import fixture as detection_fixture
from test_initial_state import owner
from test_physical_motiongen import fake_inputs
from test_real_observation import fixture as snapshot


class PlaygroundMotionGenEntryTest(unittest.TestCase):
    environment = 'physical'
    source_clock_id='cpu-fake-device-clock-v1'
    runtime_nonce='cpu-fake-runtime-v1'

    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.uploads = self.root/'uploads'; self.uploads.mkdir()
        self.remote = self.root/'remote'
        self.source, self.model, self.binding = fake_inputs()
        profile = self.source['calibration_profile']
        assets = self.root/'assets'
        urdf = assets/(ROBOT+'source/acone/urdf/a1-cal.urdf')
        usd = assets/(ROBOT+'sim/a1-cal.usd')
        for path in (urdf, usd):
            path.parent.mkdir(parents=True, exist_ok=True); path.write_text('CPU fixture '+path.suffix)
        for stem in ('acone', 'acone_no_handle', 'acone_no_handle_d405'):
            urdf.with_name(stem+'.urdf').write_bytes(urdf.read_bytes())
        for stem in ('ac_one', 'ac_one_no_handle', 'ac_one_no_handle_d405'):
            usd.with_name(stem+'.usd').write_bytes(usd.read_bytes())
        profile['model_binding']['urdf_sha256'] = sha256(urdf)
        self.binding['calibration_profile_digest'] = _digest(profile)
        binding_path, rig = assets/'a1-binding.json', assets/(ROBOT+'d405_rig.json')
        self.write(binding_path, dict(schema='a1_isaac_assets.v1', conversion_complete=True,
            profile_id=profile['profile_id'], model_binding=profile['model_binding'],
            urdf=str(urdf), usd=str(usd), usd_sha256=sha256(usd)))
        self.write(rig, dict(id=profile['profile_id'], calibration_binding=profile['model_binding']))
        self.write(self.root/'profile.json', profile)
        self.write(self.root/'binding.json', self.binding)
        if self.environment == 'fake':
            profile['source']['simulation_only'] = True
            self.binding.update(environment='fake', simulation_only=True, evidence_source='fake_device_rgbd',
                                calibration_profile_digest=_digest(profile))
            self.write(self.root/'profile.json', profile)
            self.write(self.root/'binding.json', self.binding)
        run = '/mnt/data/feiyu.zhou/runs/robot_sim/prepare-cpu-fixture'
        self.contract = dict(schema_version='anyverse.dsw-codex-experiment.v1',
            filesystem=dict(readonly_roots=[str(self.root)], writable_roots=[run, str(self.uploads)],
                            allow_overwrite=False, allow_delete=False),
            resources=dict(lanes={v['instance_id']:[v['gpu']] for v in PIPELINE_LANES.values()},
                max_gpu_count_per_instance=1, max_cpu_threads=4, require_gpu_idle_check=True, max_wall_time_minutes=20),
            provenance=dict(input_sha256={str(p):sha256(p) for p in (urdf, usd, binding_path, rig)}))
        self.cfg = {key:str(self.root/key) for key in PATH_KEYS}
        self.cfg.update(contract_path=str(self.root/'contract.json'), remote_run_root=run,
            assets=str(assets), opening_mm=83., finger_envelope_sha256='a'*64,
            planning_backend='motiongen', curobo_root=str(self.root/'curobo'))
        if self.environment=='fake':
            self.cfg.update(environment='fake',resident_deadline_wall_ns=time.time_ns()+1_200_000_000_000)
            self.contract['resources']['resident_deadline_wall_ns']=self.cfg['resident_deadline_wall_ns']
        self.write(self.root/'contract.json', self.contract)
        self.write(self.root/'config.json', self.cfg)
        self.bridge = SimpleNamespace(config=dict(preparation=dict(pipeline_config=str(self.root/'config.json'),
            acquisition=dict(profile_path=str(self.root/'profile.json'), acquisition_binding_path=str(self.root/'binding.json'),
                measured_initial_state=True, transport=dict(remote_root=str(self.uploads), endpoint=None)))))
        if self.environment=='fake':self.bridge.config['environment']='fake'
        self.last_source_ns=1_000_000_000
        def update(meta):
            ns=meta['frame_number']*1_000_000_000+1_000_000_000;self.last_source_ns=ns
            meta.update(environment='fake',simulation_only=True,evidence_source='fake_device_rgbd',timestamp_kind='simulation_step',
                clock_domain='simulation',source_clock_id=self.source_clock_id,source_time_ns=ns,physics_step=ns*240//1_000_000_000,
                host_receive_ns=time.time_ns(),device_timestamp_ms=ns/1e6,depth_device_timestamp_ms=ns/1e6,timestamp_domain='simulation',
                owner_available=True,owner_pid=1234,command_epoch=3,runtime_nonce=self.runtime_nonce)
        frames = iter(snapshot(i,update if self.environment=='fake' else None) for i in range(9))
        self.bridge.camera = lambda *args: (next(frames), 'application/zip')
        def read_owner():
            result=owner(stamp=time.time_ns())
            if self.environment=='fake':
                result.update(mode='simulation',environment='fake',evidence_source='fake',runtime_nonce=self.runtime_nonce)
                result['calibration_feedback'].update(clock_domain='simulation',source_clock_id=self.source_clock_id,
                    source_time_ns=self.last_source_ns,sample_time_s=self.last_source_ns/1e9,physics_step=self.last_source_ns*240//1_000_000_000)
            return result
        self.bridge.owner = read_owner
        self.stages = []; self.tamper = None

    @staticmethod
    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))

    def acquire(self, *args, **kwargs):
        # Real acquisition + local transport; optionally corrupt its sealed output.
        prepared = prepare_inputs(*args, **kwargs)
        if self.tamper == 'missing-pin':
            cfg, contract, _ = load_config(Path(prepared['pipeline_config']))
            contract['provenance']['input_sha256'].pop(prepared['physical_binding'])
            self.write(Path(cfg['contract_path']), contract)
        self.prepared = prepared
        return prepared

    def run_remote(self, stages, output, contract, *args):
        """Local transport double executes the production CPU binding functions."""
        cfg, _, _ = load_config(Path(self.prepared['pipeline_config']))
        pins = contract['provenance']['input_sha256']
        for stage in stages:
            name = stage['name']; self.stages.append(name)
            command = shlex.split(stage['argv'][-1])[-1]
            if name.startswith('preflight-'):
                self.assertIn('contract file SHA mismatch', command)
                for path, expected in pins.items():
                    self.assertEqual(sha256(path), expected)
            elif name == 'initialize':
                self.remote.mkdir()
            elif name == 'shared-storage':
                self.assertTrue(self.remote.is_dir())
            elif name == 'physical-capture-import':
                self.assertIn('import_capture', command)
                import_capture(self.prepared['capture_dir'], self.prepared['physical_binding'], self.remote/'capture',self.environment)
            elif name == 'detect':
                # Fake detector output refers to the actual sealed capture bytes.
                summary, rows, _ = detection_fixture()
                manifest = json.loads((self.remote/'capture/rgbd-manifest.json').read_text())
                summary['manifest_sha256'] = sha256(self.remote/'capture/rgbd-manifest.json')
                summary['inputs'] = manifest['frames']
                summary.update(valid_pair_frames=len(manifest['frames']), valid_pair_span_s=8.)
                rows = [deepcopy(rows[0]) for _ in manifest['frames']]
                for row, frame in zip(rows, manifest['frames']):
                    row.update(source_stamp_ns=frame['source_stamp_ns'], source_sequence_id=frame['capture_sequence_id'],
                               capture_sequence_id=frame['capture_sequence_id'])
                self.write(self.remote/'detection/summary.json', summary)
                (self.remote/'detection/observations.jsonl').write_text('\n'.join(map(json.dumps, rows)))
            elif name == 'physical-detection-bind':
                self.assertIn('bind_detection', command); bind_detection(self.remote,self.environment)
            elif name == 'bridge':
                tokens = shlex.split(command)
                self.assertNotIn('--simulation-initial-suite', tokens)
                self.assertEqual(tokens[tokens.index('--physical-binding')+1], cfg['remote_run_root']+'/physical-detection-binding.json')
                bound = json.loads((self.remote/'physical-detection-binding.json').read_text())
                convert(self.remote/'capture/profile.json', self.remote/'detection', self.remote/'detection-bridge',
                        tokens[tokens.index('--run-id')+1], bound, environment=self.environment)
                if self.environment=='fake':self.assertIn('--environment fake',command)
            elif name == 'physical-model-bind':
                self.assertIn('validate_assets', command)
                validate_assets(cfg['assets'], self.remote/'capture/profile.json', pins)
            elif name == 'playground-geometry-inputs':
                self.assertEqual(self.stages[-2], 'physical-model-bind')
                self.assertIn('--hardware-binding '+cfg['remote_run_root']+'/detection-bridge/hardware-binding.json', command)
                self.write(self.remote/'playground-scene/planning-input.json', dict(compute_boundary='synthetic'))
                (self.remote/'playground-scene/first-frame.png').write_bytes(b'fake-render')
            elif name == 'playground-plan':
                self.assertEqual(self.stages[-2], 'playground-geometry-inputs')
                self.assertIn('/motiongen_plan.py', command)
                if self.environment=='fake':self.assertIn('--environment fake',command)
                self.assertIn('--physical-binding '+cfg['remote_run_root']+'/detection-bridge/hardware-binding.json', command)
                self.assertEqual(stage['expected_native_status'], 'motiongen-plan-ready')
                self.synthetic_gpu_result()
            else:
                self.fail('unexpected simulation/rollout stage: '+name)

    def synthetic_gpu_result(self):
        source = json.loads((self.remote/'detection-bridge/source.json').read_text())
        binding = json.loads((self.remote/'detection-bridge/hardware-binding.json').read_text())
        model = deepcopy(self.model)
        start = source['initial_state']['model']; closed = deepcopy(start)
        closed['left_gripper'] = 65.-gripper_offset_mm(source['calibration_profile'])
        for row, target in zip(model['ticks'], (start, closed, start)):
            row['target'] = deepcopy(target)
        model.update(source_run_id=source['run_id'], source_sequence=source['sequence'],
            scene_digest=source['scene']['scene_digest'], calibration_profile_digest=_digest(source['calibration_profile']),
            native_actions=[target_vector(row['target']) for row in model['ticks']], timeline_digest=_digest(model['ticks']))
        model['planning_backend']['native_robot_sha256'] = source['calibration_profile']['model_binding']['urdf_sha256']
        model['planning_validation'].update(source_digest=_digest(source), timeline_digest=model['timeline_digest'])
        model['control_validation'] = validate_control(model)
        from test_physical_motiongen import bind_contact_fixture
        bind_contact_fixture(model,source)
        physical = freeze_physical(model, source, binding,environment=self.environment)
        self.write(self.remote/'playground-plan/plan.json', physical)
        self.write(self.remote/'playground-plan/result.json', dict(status='motiongen-plan-ready'))
        if self.tamper == 'returned-binding':
            binding['camera_serials']['left'] = 'another-camera'
            self.write(self.remote/'detection-bridge/hardware-binding.json', binding)

    def fetch(self, remote, local, **kwargs):
        cfg, _, _ = load_config(Path(self.prepared['pipeline_config']))
        shutil.copyfile(self.remote/Path(remote).relative_to(cfg['remote_run_root']), local)

    def plan(self):
        def fast_capture(*args, **kwargs):
            clock = [0.]
            self.capture_options=deepcopy(kwargs)
            return capture(*args, **kwargs, monotonic=lambda:clock[0], sleep=lambda s:clock.__setitem__(0, clock[0]+s))
        with ExitStack() as stack:
            stack.enter_context(patch('real_observation.capture', fast_capture))
            stack.enter_context(patch('real_acquisition.prepare_inputs', self.acquire))
            stack.enter_context(patch.object(pipeline, 'run_stages', self.run_remote))
            stack.enter_context(patch.object(pipeline, 'fetch', self.fetch))
            return pipeline.plan(dict(pipeline_config=str(self.root/'config.json')), self.root/'job', self.bridge, .3)

    def test_actual_caller_prepares_bound_motiongen_scene(self):
        result = self.plan()
        self.assertTrue(result['validated'])
        if self.environment=='fake':
            self.assertEqual(self.capture_options['wall_deadline_ns'],self.cfg['resident_deadline_wall_ns']-45_000_000_000)
            self.assertNotIn('timeout_s',self.capture_options)  # Retain independent 30 s source-stall watchdog.
        self.assertEqual(self.stages, ['preflight-simulated-real', 'preflight-dsw-scene', 'initialize', 'shared-storage',
            'physical-capture-import', 'detect', 'physical-detection-bind', 'bridge', 'physical-model-bind',
            'playground-geometry-inputs', 'playground-plan'])
        value = json.loads(Path(result['plan_path']).read_text())
        self.assertEqual(value['planning_validation']['schema_version'], 'physical_curobo_motiongen_path_validation.v1')
        self.assertFalse(value['planning_validation']['hardware_validated'])

    def test_unbound_capture_rejected_by_actual_caller(self):
        self.write(self.root/'binding.json', {**self.binding, 'calibration_accepted':False})
        with self.assertRaisesRegex(ValueError, 'accepted physical calibration'):
            self.plan()
        self.assertEqual(self.stages, [])

    def test_unpinned_capture_rejected_by_actual_caller(self):
        self.tamper = 'missing-pin'
        with self.assertRaisesRegex(ValueError, 'SHA-pinned readonly inputs'):
            self.plan()
        self.assertEqual(self.stages, [])

    def test_forced_diagnostic_rejected_by_actual_caller(self):
        self.cfg.update(force_diagnostic=True, diagnostic_input_root=str(self.root/'prior'))
        self.write(self.root/'config.json', self.cfg)
        with self.assertRaisesRegex(ValueError, 'cannot reuse forced diagnostic'):
            self.plan()
        self.assertEqual(self.stages, [])

    def test_complete_motiongen_cli_still_rejected(self):
        cfg, contract, _ = load_config(self.root/'config.json')
        with self.assertRaisesRegex(ValueError, 'MotionGen'):
            preparation_stages(cfg, contract, self.uploads, self.uploads/'binding.json')

    def test_unpinned_model_stops_before_geometry(self):
        self.contract['provenance']['input_sha256'].pop(str(self.root/'assets/a1-binding.json'))
        self.write(self.root/'contract.json', self.contract)
        with self.assertRaisesRegex(ValueError, 'missing SHA pin for A1 asset'):
            self.plan()
        self.assertEqual(self.stages[-1], 'physical-model-bind')

    def test_returned_plan_requires_same_physical_binding(self):
        self.tamper = 'returned-binding'
        with self.assertRaisesRegex(ValueError, 'camera serials differ'):
            self.plan()
        self.assertEqual(self.stages[-1], 'playground-plan')

    def test_reused_simulation_input_rejected(self):
        self.cfg['observation_input_root'] = str(self.root/'old-simulation')
        self.write(self.root/'config.json', self.cfg)
        with self.assertRaisesRegex(ValueError, 'not reused simulation inputs'):
            self.plan()
        self.assertEqual(self.stages, [])


if __name__ == '__main__':
    unittest.main()
