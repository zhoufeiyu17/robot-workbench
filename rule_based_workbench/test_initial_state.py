"""CPU measured-start provenance, clamp, endpoint and detector-bridge checks."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from contract import demo_state
from integration import _digest
from joint_space import validate, validate_initial_state
from real_observation import read_initial_sample, seal_initial_state
from test_joint_space import a1_profile, physical_fixture
from test_frozen_executor import fixture


def owner(state=None, *, stamp=1_000_000_000, epoch=3):
    return dict(pid=1234, command_epoch=epoch, mode='hardware', connected=True,
                feedback_healthy=True, stationary=True, automatic_enabled=False,
                state='STOPPED_DAMPING_HOLD', sample_age_s=.003,
                physical_feedback=state or demo_state(),
                calibration_feedback=dict(sample_time_s=stamp/1e9, sample_wall_ns=stamp))


def initial_fixture(profile):
    state = demo_state(); state['left_gripper']=83.095; state['right_gripper']=83.03
    state['left_arm_joints'][0] = .12
    before=read_initial_sample(lambda:owner(state),wall_ns=lambda:1_003_000_000)
    after=read_initial_sample(lambda:owner(state,stamp=9_000_000_000),wall_ns=lambda:9_003_000_000)
    return seal_initial_state(before, after, profile)


class InitialStateTest(unittest.TestCase):
    def test_measured_preserved_clamp_and_contract(self):
        source, plan = fixture()
        profile = a1_profile(source['calibration_profile'])
        profile['joints']['zero_offsets_rad'][0] = .02
        initial = initial_fixture(profile)
        self.assertEqual(initial['measured_reported']['left_gripper'],83.095)
        self.assertEqual(initial['planned_reported']['left_gripper'],83.)
        self.assertAlmostEqual(initial['model']['left_gripper'],82.992)
        self.assertAlmostEqual(initial['model']['left_arm_joints'][0],.14)
        source.update(schema_version='physical_collection_source.v1',calibration_profile=profile,initial_state=initial)
        for tick in plan['ticks']:tick['target']=deepcopy(initial['model'])
        physical_fixture(plan,source)
        plan['initial_state_digest']=_digest(initial)
        validate(plan,source)
        for index in (0,-1):
            bad=deepcopy(plan);bad['ticks'][index]['target']['left_arm_joints'][0]+=.011
            with self.assertRaisesRegex(ValueError,'first target.*return within'):validate(bad,source)
        bad=deepcopy(plan);bad['initial_state_digest']='wrong'
        with self.assertRaisesRegex(ValueError,'sealed observation'):validate(bad,source)
        for mutate in (lambda x:x['planned_reported']['left_arm_joints'].__setitem__(0,.13),
                       lambda x:x['measured_reported'].__setitem__('left_gripper',85.1),
                       lambda x:x['owner_samples'][1].__setitem__('command_epoch',4),
                       lambda x:x['after_observation_reported']['left_arm_joints'].__setitem__(0,.2)):
            bad=deepcopy(initial);mutate(bad)
            with self.assertRaises(ValueError):validate_initial_state(bad,profile)

    def test_negative_measured_gap_preserved_and_legacy_upper_clamp(self):
        source, _ = fixture()
        profile = a1_profile(source['calibration_profile'])
        state = demo_state(); state['left_gripper']=-1.; state['right_gripper']=84.
        before=read_initial_sample(lambda:owner(state),wall_ns=lambda:1_003_000_000)
        after=read_initial_sample(lambda:owner(state,stamp=9_000_000_000),wall_ns=lambda:9_003_000_000)
        value=seal_initial_state(before,after,profile)
        self.assertEqual(value['measured_reported']['left_gripper'],-1.)
        self.assertEqual(value['planned_reported']['left_gripper'],.008)
        self.assertEqual(value['model']['left_gripper'],0.)
        old=initial_fixture(profile)
        old['jaw_normalization']='clamp_to_profile_upper_limit_within_2mm'
        validate_initial_state(old,profile)

    def test_owner_rejects_stale_unhealthy_or_motion(self):
        for update in ({'sample_age_s':.101},{'automatic_enabled':True},{'feedback_healthy':False},
                       {'stationary':False},{'mode':'simulation'},{'motion':{'status':'VERIFYING'}}):
            with self.assertRaisesRegex(ValueError,'fresh healthy'):
                read_initial_sample(lambda:owner()|update,wall_ns=lambda:1_003_000_000)
        with self.assertRaises(ValueError):
            read_initial_sample(lambda:owner(),wall_ns=lambda:1_501_000_000)

    def test_detection_suite_carries_measured_start(self):
        from source_from_detection import convert
        from test_detection_bridge import fixture as detector_fixture
        summary,rows,profile=detector_fixture()
        profile=a1_profile(profile)
        initial=initial_fixture(profile)
        with TemporaryDirectory() as d:
            root=Path(d);detection=root/'detection';detection.mkdir()
            (root/'profile.json').write_text(json.dumps(profile))
            (detection/'summary.json').write_text(json.dumps(summary))
            (detection/'observations.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
            binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
                camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile),
                initial_state=initial,detection_files={k:hashlib.sha256((detection/f).read_bytes()).hexdigest()
                for k,f in [('summary_sha256','summary.json'),('observations_sha256','observations.jsonl')]})
            source=convert(root/'profile.json',detection,root/'out','run',binding)
            suite=json.loads((root/'out/suite.json').read_text())
            self.assertEqual(source['initial_state'],initial)
            self.assertEqual(suite['episodes'][0]['metadata']['initial_robot_state'],initial['model'])
            self.assertEqual(json.loads((root/'out/hardware-binding.json').read_text())['source_digest'],_digest(source))


if __name__=='__main__':unittest.main()
