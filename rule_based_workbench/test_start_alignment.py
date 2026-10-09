"""CPU-only bounded alignment protocol, using the real calibration MotionRunner."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
from unittest.mock import patch

from integration import _digest
from real_control import classes, load_calibration
from test_real_control import State, Takeover
from test_joint_space import a1_profile
from test_frozen_executor import fixture
import jaw_mapping_v2 as jaw


def rejects(call):
    try:
        call()
    except (ValueError, KeyError):
        return
    raise AssertionError('unsafe command accepted')


def main(root, motion_patch=None):
    ref = load_calibration(root)
    if motion_patch is not None:
        name = 'arx_env.joint_head_real_motion'
        spec = importlib.util.spec_from_file_location(name, motion_patch)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        ref.MotionRunner = module.MotionRunner
    from arx_env.joint_head_jaw_mapping import configure_mapping
    from arx_env.joint_head_real_motion import MotionRunner
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        mapping = tmp/'jaw.json'
        mapping.write_text(json.dumps(dict(schema_version='jaw_mapping_audit.v1', reinterpreted_mapping={
            side: dict(candidate_use_allowed=True, command_sdk_knots=list(jaw.COMMAND[side]),
                       feedback_sdk_knots=list(jaw.FEEDBACK[side]), metal_knots_mm=list(jaw.GAP_MM[side]))
            for side in ('left','right')})))
        profile = a1_profile(fixture()[0]['calibration_profile'])
        profile['source'] = {'kind':'installed_physical_calibration'}
        profile['joints']['zero_offsets_rad'] = [.01,-.02,.015,0.,.005,-.01]*2
        profile['model_binding']['jaw_mapping_sha256'] = hashlib.sha256(mapping.read_bytes()).hexdigest()
        (tmp/'profile.json').write_text(json.dumps(profile))
        binding = dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fake-arx',
                       camera_serials=dict(head='head',left='left',right='right'), calibration_profile_digest=_digest(profile))
        (tmp/'binding.json').write_text(json.dumps(binding))
        Owner, _, _ = classes(ref, jaw_mapping_file=mapping)
        safety = SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State)
        clock = [100.]
        with patch('time.monotonic', lambda:clock[0]):
            def create():
                backend = ref.SimulatedBackend([0.]*6+[.04]+[0.]*6+[.04])
                owner = Owner(backend,safety,hardware_binding=tmp/'binding.json',alignment_profile=tmp/'profile.json')
                for _ in range(42):owner.tick();clock[0]+=.01
                return owner, backend
            owner, backend = create()
            events = list(backend.events)
            for arm in ('both','invalid'):
                rejects(lambda:owner.submit(dict(action='preview_start_alignment',active_arm=arm)))
            rejects(lambda:owner.submit(dict(action='preview_start_alignment',active_arm='left',q_target=[0.]*14)))
            review = owner.submit(dict(action='preview_start_alignment',active_arm='left'))
            plan = owner.pending[review['token']][0][0]
            assert backend.events == events
            assert plan.max_velocity == .01 and plan.max_delta <= .05
            assert plan.q_target[:6] == tuple(-v for v in profile['joints']['zero_offsets_rad'][:6])
            assert plan.q_start[6:] == plan.q_target[6:], 'jaws/inactive arm moved'
            assert plan.duration_s >= 1.875*plan.max_delta/.01
            rejects(lambda:owner.submit(dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=False)))
            owner.submit(dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
            owner.tick()
            assert isinstance(owner.runner, MotionRunner) and not hasattr(owner.runner,'frozen')
            for _ in range(600):
                backend.q[6] = backend.q[13] = .04  # Held physical jaw feedback, not command units.
                clock[0]+=.01;owner.tick()
            assert owner.runner.status == 'READY', (owner.runner.status,owner.runner.reason)
            assert tuple(owner.runner.execution_target[:6]) == plan.q_target[:6], 'reviewed arm target was silently replaced'
            assert all(abs(actual-target) < 1e-9 for actual,target in zip(backend.q[:6],plan.q_target[:6])), 'READY without reaching target'
            assert any(abs(q) > .005 for q in backend.q[:6]), 'nonzero target requires actual movement'
            assert backend.q[7:13] == [0.]*6, 'inactive arm moved'
            owner.snapshot()  # Alignment runner must not be treated as FrozenExecutor.
            rejects(lambda:owner.start_motion(plan, owner.cache['calibration_feedback']))
            owner, backend = create()
            review = owner.submit(dict(action='preview_start_alignment',active_arm='right'))
            plan = owner.pending[review['token']][0][0]
            owner.submit(dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
            for _ in range(600):
                backend.q[6] = backend.q[13] = .04
                owner.tick();clock[0]+=.01
            assert owner.runner.status == 'READY'
            assert tuple(owner.runner.execution_target[7:13]) == plan.q_target[7:13]
            assert all(abs(actual-target)<1e-9 for actual,target in zip(backend.q[7:13],plan.q_target[7:13]))
            assert backend.q[:6] == [0.]*6 and any(abs(q)>.005 for q in backend.q[7:13])
            owner, backend = create()
            owner.cache['calibration_feedback']['qpos'][7] = .1
            rejects(lambda:owner.submit(dict(action='preview_start_alignment',active_arm='right')))
            owner, backend = create()
            clock[0]+=.2
            rejects(lambda:owner.submit(dict(action='preview_start_alignment',active_arm='left')))
            owner, backend = create()
            review=owner.submit(dict(action='preview_start_alignment',active_arm='right'))
            assert review['plan']['q_target'][:7] == review['plan']['q_start'][:7]
            with patch.object(owner,'pedal_snapshot',return_value=(1,True,True)):
                rejects(lambda:owner.submit(dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True)))
            owner, backend = create()
            review=owner.submit(dict(action='preview_start_alignment',active_arm='right'))
            clock[0]+=31
            rejects(lambda:owner.submit(dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True)))
            owner, backend = create()
            review=owner.submit(dict(action='preview_start_alignment',active_arm='right'))
            owner.submit(dict(action='stop'))
            rejects(lambda:owner.submit(dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True)))
            assert not owner.alignment_plans
            binding['calibration_profile_digest']='0'*64
            (tmp/'binding.json').write_text(json.dumps(binding))
            rejects(create)
        configure_mapping(None)
    print('alignment: trusted profile, bounded single-arm motion, held jaws, no preview sends, confirmation/stop/stale gates passed')


if __name__ == '__main__':
    main(Path(sys.argv[1]), Path(sys.argv[2]) if len(sys.argv)>2 else None)
