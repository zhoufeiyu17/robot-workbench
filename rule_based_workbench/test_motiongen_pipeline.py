"""Fake-only dual-DSW MotionGen wiring; never GPU/SSH/hardware completion evidence."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shlex
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import sys
from unittest.mock import patch

from export_plan import publish_motiongen
from gripper_distance_plan import HOLDING_STAGES, NATIVE_DISTANCE_ENCODING
from integration import CUROBO_PLANNER_ID, DualGpuPipeline, PIPELINE_LANES, SIM_POLICY_ID, _digest
from isaac_lane import SimDistanceActionSpec, install_distance_replay, native_target, replay_telemetry, validate_geometry_inputs, validate_replay_inputs
from real_pipeline import preparation_stages
from run_pipeline import PATH_KEYS, build_plan, load_config
from test_app import fixture_messages


def rejects(function):
    try:
        function()
    except (ValueError, FileExistsError):
        return
    raise AssertionError('invalid fixture accepted')


def motiongen_fixture():
    source, old, _ = fixture_messages()
    source.update(adapter='foundationpose', observation_mode='offline_replay', test_fixture=True)
    source['calibration_profile']['gripper'].update(mapping='sum_of_two_prismatic_displacements_mm', max_opening_mm=83.)
    source['calibration_profile']['source'] = dict(kind='simulation-authored', hardware_calibrated=False)
    plan = deepcopy(old)
    plan.update(planner_id=CUROBO_PLANNER_ID, policy_id=SIM_POLICY_ID,
        calibration_profile_digest=_digest(source['calibration_profile']), gripper_opening_mm=83.,
        control=dict(physics_hz=240, substeps=8, max_target_velocity_rad_s=2., max_target_acceleration_rad_s2=5., episode_timeout_s=300.),
        simulation=dict(validated=False, outcome='not_run'), native_action_encoding=NATIVE_DISTANCE_ENCODING,
        planning_backend=dict(backend='curobo_motiongen', curobo_version='0.7.8', gpu_ik=True, gpu_trajectory_search=True),
        gripper_distance_control=dict(schema_version='sim_gripper_distance_control.v1', closed_metal_gap_mm=65.,
            rubber_gap_mm=62., pad_thickness_m=.0015, native_travel_offset_mm=.008, holding_stages=sorted(HOLDING_STAGES)),
        stages=[dict(name='approach', start_tick=0), dict(name='close_and_hold', start_tick=1),
                dict(name='open', start_tick=2)], native_substeps=[8, 8, 8], duration_s=.1)
    target = deepcopy(old['ticks'][0]['target'])
    target.update(left_arm_joints=[0.]*6, right_arm_joints=[0.]*6, right_gripper=83.)
    plan['ticks'] = [dict(tick=i, time_s=i/30, target={**deepcopy(target), 'left_gripper': gap})
                     for i, gap in enumerate((83., 64.992, 83.))]
    plan['native_actions'] = [[0.]*6+[row['target']['left_gripper']]+[0.]*6+[83.] for row in plan['ticks']]
    plan['timeline_digest'] = _digest(plan['ticks'])
    plan['planning_validation'] = dict(schema_version='curobo_motiongen_path_validation.v1', passed=True,
        source_digest=_digest(source), timeline_digest=plan['timeline_digest'])
    from test_physical_motiongen import bind_contact_fixture
    bind_contact_fixture(plan,source)
    return source, plan


def main():
    # A growing target below the rigid contact gap unloads grip preload; the
    # original release force guard becomes active at contact, without new targets.
    release=SimDistanceActionSpec(None,82.992,release_contact_mm=76.142)
    raw=[0.]*6+[64.992]+[0.]*6+[82.992]
    release.set_stage('open');assert release._binary(raw)[6]==-1.
    raw[6]=75.;assert release._binary(raw)[6]==-1.
    raw[6]=76.142;assert release._binary(raw)[6]==1.
    release.set_stage('close_and_hold');assert release._binary(raw)[6]==-1.
    rejects(lambda:SimDistanceActionSpec(None,82.992,release_contact_mm=90.))
    with TemporaryDirectory() as temporary:
        folder = Path(temporary)
        remote = '/mnt/data/feiyu.zhou/runs/robot_sim/motiongen-fixture'
        contract = dict(schema_version='anyverse.dsw-codex-experiment.v1',
            filesystem=dict(readonly_roots=['/fixture/input'], writable_roots=[remote], allow_overwrite=False, allow_delete=False),
            resources=dict(lanes={v['instance_id']: [v['gpu']] for v in PIPELINE_LANES.values()},
                max_gpu_count_per_instance=1, max_cpu_threads=4, require_gpu_idle_check=True, max_wall_time_minutes=20))
        contract_path = folder/'contract.json'; contract_path.write_text(json.dumps(contract))
        config = {key: '/fixture/input/'+key for key in PATH_KEYS}
        config.update(contract_path=str(contract_path), remote_run_root=remote, finger_envelope_sha256='a'*64,
                      opening_mm=83., planning_backend='motiongen', curobo_root='/fixture/input/curobo')
        config_path = folder/'config.json'; config_path.write_text(json.dumps(config))
        configured, _, _ = load_config(config_path)
        stages = build_plan(configured, contract)
        names = [stage['name'] for stage in stages]
        expected = ['observe', 'observe-verify', 'detect', 'bridge', 'motiongen-scene', 'motiongen-plan',
                    'motiongen-publish', 'replay-source', 'replay-target']
        assert [name for name in names if name in expected] == expected
        assert not {'geometry', 'geometry-verify', 'place', 'export'} & set(names)
        mapping = {stage['name']: stage for stage in stages}
        assert all(mapping[name]['side'] == 'simulated-real' for name in ('observe', 'detect', 'bridge'))
        assert all(mapping[name]['side'] == 'dsw-scene' for name in ('motiongen-scene', 'motiongen-plan', 'motiongen-publish'))
        commands = {name: shlex.split(stage['argv'][-1])[-1] for name, stage in mapping.items()}
        assert '--mode geometry-inputs' in commands['motiongen-scene']
        assert '--source '+remote+'/detection-bridge/source.json' in commands['motiongen-scene']
        assert 'motiongen_plan.py' in commands['motiongen-plan'] and '--speed-scale 1.0' in commands['motiongen-plan']
        assert '--input '+remote+'/motiongen-scene/planning-input.json' in commands['motiongen-plan']
        assert 'CUDA_VISIBLE_DEVICES=0' in commands['motiongen-plan']
        assert '/fixture/input/curobo/python:/fixture/input/curobo/deps:' in commands['motiongen-plan']
        for name in ('replay-source', 'replay-target'):
            assert '--plan '+remote+'/plan.json' in commands[name]
            assert '--source '+remote+'/detection-bridge/source.json' in commands[name]
            assert '--hardware-binding' not in commands[name] and '--independent-replay' in commands[name]
        assert '--suite '+config['initial_suite'] in commands['replay-source']
        assert '--suite '+remote+'/detection-bridge/suite.json' in commands['replay-target']
        assert not any('playground_geometry.py' in command or '--mode place' in command for command in commands.values())
        native = build_plan({**config, 'planning_backend': 'native'}, contract)
        assert {'geometry', 'place', 'export'} <= {stage['name'] for stage in native}
        rejects(lambda: preparation_stages(config, contract, '/not-read', '/not-read/binding.json'))

        source, plan = motiongen_fixture()
        bridge = DualGpuPipeline(); bridge.ingest_source(source); bridge.accept_plan(plan)
        for key in ('grasp_contact_policy','grasp_contact_certificate'):
            missing=deepcopy(plan);missing.pop(key)
            rejects(lambda:bridge.accept_plan(missing))
        for change in (dict(offline_candidate=True), dict(planning_backend={'backend': 'curobo_world_broadphase_native_exact'}),
                      dict(simulation={'validated': True, 'outcome': 'success'}), dict(native_action_encoding='binary')):
            rejects(lambda change=change: bridge.accept_plan({**plan, **change}))
        changed = deepcopy(plan); changed['ticks'][1]['target']['left_gripper'] = 0.
        changed['native_actions'][1][6] = 0.; changed['timeline_digest'] = _digest(changed['ticks'])
        changed['planning_validation']['timeline_digest'] = changed['timeline_digest']
        rejects(lambda: bridge.accept_plan(changed))
        physical = deepcopy(source); physical.update(schema_version='physical_collection_source.v1', lane='physical-real')
        rejects(lambda: DualGpuPipeline().ingest_source(physical))
        source_path = folder/'source.json'; source_path.write_text(json.dumps(source))
        planned = folder/'planned'; planned.mkdir()
        (planned/'plan.json').write_text(json.dumps(plan))
        result = dict(status='motiongen-plan-ready', backend='curobo_motiongen', timeline_digest=plan['timeline_digest'])
        (planned/'result.json').write_text(json.dumps(result))
        publish_motiongen(source_path, planned, folder/'plan.json')
        assert json.loads((folder/'plan.json').read_text()) == plan
        rejects(lambda: publish_motiongen(source_path, planned, folder/'plan.json'))
        material_plan={**deepcopy(plan),'native_identity':{'physical_configuration':{'base':'unchanged'}}}
        (planned/'plan.json').write_text(json.dumps(material_plan))
        preset=dict(preset_id='firm',stiffness_n_m=12000.,damping_n_s_m=10.,diagnostic_claw_compression=True)
        publish_motiongen(source_path,planned,folder/'preset-plan.json',preset)
        frozen=json.loads((folder/'preset-plan.json').read_text())
        assert frozen['ticks']==plan['ticks'] and frozen['timeline_digest']==plan['timeline_digest']
        assert frozen['native_identity']['physical_configuration']['cup_contact_preset']==preset
        assert frozen['preset_qualification'].endswith('default not selected')
        (planned/'plan.json').write_text(json.dumps(plan))
        (planned/'result.json').write_text(json.dumps({**result, 'timeline_digest': 'wrong'}))
        rejects(lambda: publish_motiongen(source_path, planned, folder/'rejected.json'))
        assert not (folder/'rejected.json').exists()

        suite = dict(episodes=[source['scene']['episode_scene']])
        for side, lane in PIPELINE_LANES.items():
            args = SimpleNamespace(side=side, instance=lane['instance_id'], gpu=lane['gpu'], mode='replay')
            validate_replay_inputs(args, plan, source, suite)
            receipt = dict(schema_version='dual_gpu_pipeline.v1', side=side, **lane,
                source_run_id=source['run_id'], source_sequence=source['sequence'], scene_digest=source['scene']['scene_digest'],
                calibration_profile_digest=_digest(source['calibration_profile']), plan_run_id=plan['run_id'],
                timeline_digest=plan['timeline_digest'], status='completed', tick_count=len(plan['ticks']),
                applied_ticks=deepcopy(plan['ticks']), measured_states=[row['target'] for row in plan['ticks']])
            bridge.record_execution(receipt)
            broken = deepcopy(receipt); broken['applied_ticks'][1]['target']['left_gripper'] += 1.
            rejects(lambda: bridge.record_execution(broken))
        assert bridge.summary()['execution'] == 'replay-complete'
        envelope = folder/'finger-envelope.json'; envelope.write_text('{}')
        args = SimpleNamespace(side='dsw-scene', finger_envelope=envelope,
                               finger_envelope_sha256=hashlib.sha256(envelope.read_bytes()).hexdigest())
        validate_geometry_inputs(args, source, suite)
        rejects(lambda: validate_geometry_inputs(args, physical, suite))

    class NativeSpec:
        def decode(self, raw):
            assert raw[6] in (-1., 1.) and raw[13] in (-1., 1.), 'distance value reached the binary decoder'
            if any(not -1 <= value <= 1 for value in raw): raise ValueError('joint limit')
            return SimpleNamespace(left=SimpleNamespace(joint_positions_rad=raw[:6], gripper_open=raw[6]>0),
                                   right=SimpleNamespace(joint_positions_rad=raw[7:13], gripper_open=raw[13]>0))
        def decode_single_arm_transition(self, old, new, **kwargs):
            self.decode(old)
            if old[7:] != new[7:]: raise ValueError('inactive arm')
            return self.decode(new)
    spec = NativeSpec(); adapter = SimDistanceActionSpec(spec, 83.)
    raw = [0.]*6+[64.992]+[0.]*6+[83.]
    previous = [0.]*6+[83.]+[0.]*6+[83.]
    adapter.decode_single_arm_transition(previous, raw, active_arm='left')
    for stage in ('close_and_hold','lift','transport','lower','support_settle'):
        adapter.set_stage(stage)
        assert adapter.decode(raw).left.gripper_open is False
        assert adapter.decode_single_arm_transition(previous, raw, active_arm='left').left.gripper_open is False
    adapter.set_stage('open')
    assert adapter.decode(raw).left.gripper_open is True, 'release intent must precede completion of opening ramp'
    target = native_target(raw, spec, 83., distance=True)
    assert target['left_gripper'] == 64.992 and target['right_gripper'] == 83.
    wrong = raw.copy(); wrong[13] = 82.
    rejects(lambda: adapter.decode_single_arm_transition(previous, wrong, active_arm='left'))
    wrong = raw.copy(); wrong[0] = 2.
    rejects(lambda: adapter.decode_single_arm_transition(previous, wrong, active_arm='left'))
    wrong = raw.copy(); wrong[6] = 84.
    rejects(lambda: adapter.decode(wrong))

    # The installed action term must command distances while preserving native
    # guarded execution/physics/evaluator entrypoints (all objects here are fake).
    class Matrix:
        def __init__(self, values): self.values=deepcopy(values); self.dtype='float64'
        @property
        def shape(self): return (len(self.values),len(self.values[0]))
        def __getitem__(self, key): return SimpleNamespace(tolist=lambda:deepcopy(self.values[key]))
        def __setitem__(self, key, value): self.values[key]=deepcopy(value.values)
    class Limits:
        def __getitem__(self, key):
            return SimpleNamespace(tolist=lambda:[[-2.,2.]]*6+[[0.,.0415]]*2+[[-2.,2.]]*6+[[0.,.0415]]*2)
    spec.left=SimpleNamespace(arm=SimpleNamespace(joint_names=[f'left_joint{i}' for i in range(1,7)]))
    spec.right=SimpleNamespace(arm=SimpleNamespace(joint_names=[f'right_joint1{i}' for i in range(1,7)]))
    names=[*spec.left.arm.joint_names,'left_joint7','left_joint8',*spec.right.arm.joint_names,'right_joint17','right_joint18']
    apply=object();step=object();execute=object()
    term=SimpleNamespace(ids=list(range(16)),device='fake',_raw=Matrix([previous]),_targets=Matrix([[0.]*16]),apply_actions=apply)
    robot=SimpleNamespace(joint_names=names,data=SimpleNamespace(joint_pos_limits=Limits()))
    env=SimpleNamespace(scene={'robot':robot},action_manager=SimpleNamespace(get_term=lambda key:term))
    runtime=SimpleNamespace(loop=SimpleNamespace(previous=[0.]*6+[1.]+[0.]*6+[1.],execute=execute),_advance=step)
    fake_torch=SimpleNamespace(tensor=lambda values,**kwargs:Matrix(values),isfinite=lambda values:SimpleNamespace(all=lambda:True))
    with patch.dict(sys.modules,torch=fake_torch):
        installed=install_distance_replay(env,runtime,spec,dict(gripper_opening_mm=83.,native_actions=[raw]))
        term.process_actions(Matrix([raw]))
    assert runtime.loop.action_spec is installed and runtime.action is installed
    assert runtime.loop.execute is execute and runtime._advance is step and term.apply_actions is apply
    assert term._targets.values[0][6:8]==[64.992/2000]*2
    assert term._targets.values[0][14:]==[83./2000]*2 and term._raw.values[0]==raw
    runtime=SimpleNamespace(loop=SimpleNamespace(physics_steps=0),read=lambda _: (SimpleNamespace(),None),
                            evaluator=SimpleNamespace(last_audit={},facts={}))
    for stage,expected in (('close_and_hold','holding'),('transport','holding'),('open','release_transition')):
        row=replay_telemetry(runtime,stage,{'tick':0,'time_s':0.},target,{},target,distance=True)
        assert row['grasp_state']==expected and row['grasp_state_basis']=='command_stage'
    lost=replay_telemetry(runtime,'transport',{'tick':0,'time_s':0.},target,{'event':{'reason':'grasp_lost'}},target,distance=True)
    assert lost['grasp_state']=='lost' and lost['grasp_state_basis']=='native_grasp_lost_event'
    print('MotionGen dual wiring, model-distance semantics and shared timeline: PASS (CPU fixtures only; no runtime completion claim)')


if __name__ == '__main__':
    main()
