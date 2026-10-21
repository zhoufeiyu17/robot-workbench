"""CPU source contracts only; no detection, model loading, or robot access."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from integration import _digest, _validate_scene
from real_contract import validate_real_source, validate_assisted_cup_geometry
from source_from_detection import build_assisted_source, convert_assisted, build_source
from test_physical_motiongen import fake_inputs


def receipt_fixture(path='/task-owned/raw.npz',sha='a'*64):
    source,_,binding=fake_inputs();profile=source['calibration_profile']
    receipt=dict(schema_version='operator_assisted_rgbd_scene.v1',status='operator_accepted',
        scope='single_observation',request_id='one-off-accepted-scene',accepted_wall_ns=10_000_000_000,
        robot_id=binding['robot_id'],camera_serials=deepcopy(binding['camera_serials']),
        calibration_profile_digest=_digest(profile),initial_state_digest=_digest(binding['initial_state']),
        frames=[dict(camera='head',camera_serial='h',path=path,sha256=sha,source_stamp_ns=9_000_000_000,
                     frame_id='d405_base_0_color_optical_frame')],
        machine_asset='objects/owner_coffee_machine_00003/v1',
        environment_from_machine_root=deepcopy(source['scene']['environment_from_machine_root']),
        environment_from_cup=dict(position=[.4723,.2814,.7811],orientation_wxyz=[1.,0.,0.,0.]),
        cup_geometry=dict(origin='bottom_center',units='m',dimensions_m=[.082,.082,.108],profile_sha256='b'*64,
            bands=[dict(shape_path='/Cup/Collisions/MiddleBody',z_min_m=0.,z_max_m=.108,
                        radius_m=.041,radius_start_m=.028,radius_end_m=.041)]),
        grasp_command=dict(metal_gap_mm=65.,units='metal_inner_gap_mm',controlled_gripper='left_gripper'),
        estimation=dict(method='rgbd_silhouette_known_dimensions',
                        provenance_summary='CPU fixture only; accepted shape metadata, no geometry qualification'))
    return receipt,profile,binding


class AssistedSourceTest(unittest.TestCase):
    def test_assisted_cup_reuses_only_seed_not_previous_cup_acceptance(self):
        from types import SimpleNamespace
        from motiongen_targets import REFERENCE, checked_reference
        planner=SimpleNamespace(cup=SimpleNamespace(profile_sha256='e'*64),
            identity={key:REFERENCE[key] for key in ('native_geometry_sha256','envelope_sha256')})
        data=dict(urdf_sha256=REFERENCE['urdf_sha256'])
        with self.assertRaises(ValueError):checked_reference(planner,data)
        planner.operator_assisted_cup_geometry=dict(profile_sha256='e'*64)
        ref,_=checked_reference(planner,data)
        self.assertEqual(ref['cup_from_tool_position_m'],REFERENCE['cup_from_tool_position_m'])
        self.assertFalse(ref['assistance_derivation']['contact_and_path_acceptance_reused'])
        self.assertEqual(ref['assistance_derivation']['scope'],'unverified_task_pose_seed')
        planner.identity['native_geometry_sha256']='f'*64
        with self.assertRaises(ValueError):checked_reference(planner,data)

    def test_assisted_envelope_still_checks_actual_world_geometry(self):
        from unittest.mock import patch
        from cup_envelope import prepare
        proof=json.loads((ROOT/'rule_based_workbench/a1_cup_inset.json').read_text())
        with TemporaryDirectory() as directory:
            path=Path(directory)/'operator-assisted-cup-envelope.json'
            proof['cup_geometry_sha256']='assisted-world'
            path.write_text(json.dumps(proof))
            with patch('cup_contact.cup_geometry_digest',return_value='assisted-world'):
                self.assertEqual(prepare(object(),path=path)['cup_geometry_sha256'],'assisted-world')
            with patch('cup_contact.cup_geometry_digest',return_value='old-world'):
                with self.assertRaises(ValueError):prepare(object(),path=path)

    def test_one_frame_retains_real_span_and_separate_accepted_source(self):
        receipt,profile,binding=receipt_fixture()
        source,suite,sealed=build_assisted_source(receipt,profile,'assisted-run',binding)
        validate_real_source(source,sealed)
        observation=source['scene']['source']
        self.assertEqual(observation['sample_count'],1)
        self.assertEqual(observation['window_start_ns'],observation['window_end_ns'])
        self.assertIs(observation['automatic_window_qualified'],False)
        self.assertNotIn('detection',observation)
        self.assertEqual(source['initial_state'],binding['initial_state'])
        self.assertEqual(source['cup_geometry']['dimensions_m'],[.082,.082,.108])
        self.assertEqual(set(suite['episodes'][0]['metadata']),{'initial_robot_state','initial_robot_state_digest'})
        self.assertEqual(sealed['operator_assistance_request_id'],receipt['request_id'])
        with self.assertRaisesRegex(ValueError,'explicit physical'):_validate_scene(source['scene'])
        with self.assertRaises(ValueError):validate_real_source(source,sealed,environment='fake')

    def test_receipt_identity_geometry_and_counts_cannot_be_relabelled(self):
        receipt,profile,binding=receipt_fixture()
        for mutate in (
            lambda r:r.update(status='candidate'),lambda r:r.update(scope='all_future_observations'),
            lambda r:r.update(robot_id='other'),lambda r:r['camera_serials'].update(head='other'),
            lambda r:r.update(initial_state_digest='c'*64),lambda r:r.update(calibration_profile_digest='c'*64),
            lambda r:r['cup_geometry'].update(origin='box_center'),
            lambda r:r['cup_geometry'].update(dimensions_m=[.09,.09,.13]),
            lambda r:r['cup_geometry']['bands'][0].update(radius_start_m=.05),
            lambda r:r.update(estimation={}),lambda r:r['frames'][0].update(sha256='wrong'),
            lambda r:r.update(accepted_wall_ns=1)):
            candidate=deepcopy(receipt);mutate(candidate)
            with self.assertRaises(ValueError):build_assisted_source(candidate,profile,'run',binding)
        source,_,sealed=build_assisted_source(receipt,profile,'run',binding)
        for key in ('operator_assistance_digest','operator_assistance_request_id','cup_geometry_digest'):
            bad=dict(sealed);bad.pop(key)
            with self.assertRaises(ValueError):validate_real_source(source,bad)
        bad=deepcopy(source);bad['adapter']='foundationpose'
        with self.assertRaises(ValueError):validate_real_source(bad,dict(sealed,source_digest=_digest(bad)))

    def test_actual_cup_geometry_must_match_not_only_box_metadata(self):
        receipt,profile,binding=receipt_fixture()
        source,_,_=build_assisted_source(receipt,profile,'run',binding)
        exported=dict(profile_sha256=receipt['cup_geometry']['profile_sha256'],bands=deepcopy(receipt['cup_geometry']['bands']))
        validate_assisted_cup_geometry(source,exported)
        validate_assisted_cup_geometry(source,dict(exported,bands=tuple(exported['bands'])))
        wrong=deepcopy(exported);wrong['bands'][0]['z_max_m']=.13
        with self.assertRaisesRegex(ValueError,'actual planner cup'):
            validate_assisted_cup_geometry(source,wrong)
        wrong=deepcopy(exported);wrong['profile_sha256']='c'*64
        with self.assertRaises(ValueError):validate_assisted_cup_geometry(source,wrong)

    def test_three_output_files_bind_existing_raw_without_copy_or_overwrite(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);raw=root/'frame.npz';raw.write_bytes(b'cpu raw fixture')
            receipt,profile,binding=receipt_fixture(str(raw),hashlib.sha256(raw.read_bytes()).hexdigest())
            (root/'receipt.json').write_text(json.dumps(receipt));(root/'profile.json').write_text(json.dumps(profile))
            output=root/'detection-bridge'
            source=convert_assisted(root/'profile.json',root/'receipt.json',output,'run',binding)
            self.assertEqual(sorted(p.name for p in output.iterdir()),['hardware-binding.json','source.json','suite.json'])
            self.assertEqual(json.loads((output/'source.json').read_text()),source)
            with self.assertRaises(FileExistsError):
                convert_assisted(root/'profile.json',root/'receipt.json',output,'run',binding)
            raw.write_bytes(b'tampered')
            with self.assertRaisesRegex(ValueError,'evidence file/hash'):
                convert_assisted(root/'profile.json',root/'receipt.json',root/'bad','run',binding)
            self.assertFalse((root/'bad').exists())

    def test_automatic_detection_still_needs_its_original_completed_window(self):
        from test_detection_bridge import fixture
        summary,observations,profile=fixture()
        summary=deepcopy(summary);summary['status']='OPERATOR_ACCEPTED'
        with self.assertRaisesRegex(ValueError,'DETECTION_WINDOW_READY'):
            build_source(summary,observations,profile,'not-automatic')


if __name__=='__main__':unittest.main()
