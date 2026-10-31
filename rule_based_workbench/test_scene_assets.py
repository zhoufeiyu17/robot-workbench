"""CPU binding checks; no pose, grasp or physical qualification claims."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import subprocess
import sys
from types import SimpleNamespace
import unittest

from integration import _validate_scene
from scene_assets import (digest, validate, load, selected_cup_asset, check_geometry,
                          episode_metadata, derive_grasp_command, SCHEMA, LEGACY_CUP)
from source_from_detection import build_source, convert
from test_detection_bridge import fixture


def selection(pin):
    value = dict(schema=SCHEMA,
        machine=dict(asset_id='coffee_final_machine', revision='20261022.material.1',
            native_asset='objects/coffee_final_machine/20261022.1', interaction_profile_sha256='a'*64),
        cup=dict(asset_id='white_green_cup_no_lid', revision='20261022.material.1',
            native_asset='objects/white_green_cup_no_lid/20261022.1', interaction_profile_sha256='b'*64),
        perception=dict(release_manifest_digest='sha256:'+'c'*64, runtime_config_id='sha256:'+'d'*64,
            node_config_sha256='sha256:'+'e'*64), input_sha256={str(pin):digest('fixture source')})
    value['asset_set_digest'] = digest(value)
    return value


def seal(value):
    value['asset_set_digest'] = digest({k:v for k,v in value.items() if k!='asset_set_digest'})
    return value


class SceneAssetTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.pin = self.root/'source.json'
        self.pin.write_text(json.dumps('fixture source',sort_keys=True,allow_nan=False))
        self.selected = selection(self.pin)
        (self.root/'scene-assets.json').write_text(json.dumps(self.selected))

    def detected(self):
        summary, observations, profile = fixture()
        summary.update(self.selected['perception'])
        for row in observations:
            row['runtime_config_id'] = summary['runtime_config_id']
            for track in row['tracks']:
                expected = self.selected['machine' if track['object_type']=='coffee_machine' else 'cup']
                track['provenance']['asset'] = {k:expected[k] for k in ('asset_id','revision')}
        return summary, observations, profile

    def test_actual_file_pins_and_no_missing_manifest_fallback(self):
        self.assertEqual(load(self.root,expected_digest=self.selected['asset_set_digest']),self.selected)
        self.pin.write_text('changed')
        with self.assertRaisesRegex(ValueError,'dependency changed'):
            load(self.root)
        (self.root/'scene-assets.json').unlink()
        with self.assertRaisesRegex(ValueError,'legacy fallback is forbidden'):
            load(self.root,expected_digest=self.selected['asset_set_digest'])
        self.assertEqual(selected_cup_asset(self.root,None),LEGACY_CUP)

    def test_selected_cup_envelope_is_pinned_to_file_and_profile(self):
        proof=self.root/'cup-envelope.json'
        proof.write_text(json.dumps({'cup_profile_sha256':'b'*64}))
        sha=hashlib.sha256(proof.read_bytes()).hexdigest()
        self.selected['cup']['inward_envelope']=dict(path=str(proof),sha256=sha)
        with self.assertRaisesRegex(ValueError,'immutable file pin'):
            validate(seal(self.selected))
        self.selected['input_sha256'][str(proof)]=sha
        (self.root/'scene-assets.json').write_text(json.dumps(seal(self.selected)))
        self.assertEqual(load(self.root),self.selected)
        proof.write_text(json.dumps({'cup_profile_sha256':'f'*64}))
        with self.assertRaisesRegex(ValueError,'dependency changed'):load(self.root)
        sha=hashlib.sha256(proof.read_bytes()).hexdigest()
        self.selected['cup']['inward_envelope']['sha256']=sha
        self.selected['input_sha256'][str(proof)]=sha
        (self.root/'scene-assets.json').write_text(json.dumps(seal(self.selected)))
        with self.assertRaisesRegex(ValueError,'another profile'):load(self.root)

    def test_rim_feature_is_pinned_and_bridge_requires_its_receipt(self):
        from cup_pose_refinement import METHOD,mesh_rim
        import math
        mesh=self.root/'rim.obj'
        mesh.write_text(''.join(f'v {.0395*math.cos(i*math.pi/32)} {.0395*math.sin(i*math.pi/32)} {z}\n'
            for z in (0.,.10925)for i in range(64)))
        sha=hashlib.sha256(mesh.read_bytes()).hexdigest()
        refinement=dict(schema='cup_pose_refinement.v1',method=METHOD,cup_profile_sha256='b'*64,
            perception_asset_digest='c'*64,mesh_sha256=sha,**mesh_rim(mesh),
            max_fit_rms_px=.75,max_position_delta_m=.03)
        self.selected['cup']['pose_refinement']=refinement
        with self.assertRaisesRegex(ValueError,'visual mesh file pin'):validate(seal(self.selected))
        self.selected['input_sha256'][str(mesh)]=sha
        (self.root/'scene-assets.json').write_text(json.dumps(seal(self.selected)))
        self.assertEqual(load(self.root),self.selected)
        probe='import sys;sys.path.insert(0,sys.argv[1]);from scene_assets import load;load(sys.argv[2]);assert "numpy"not in sys.modules'
        result=subprocess.run([sys.executable,'-S','-B','-c',probe,str(Path(__file__).parent),str(self.root)],
            capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        summary,observations,profile=self.detected()
        with self.assertRaisesRegex(ValueError,'refinement receipt missing'):
            build_source(summary,observations,profile,'missing-refinement',asset_set=self.selected)
        self.selected['cup']['pose_refinement']['radius_m']+=.001
        (self.root/'scene-assets.json').write_text(json.dumps(seal(self.selected)))
        with self.assertRaisesRegex(ValueError,'visual mesh feature'):load(self.root)

    def test_observation_and_native_consumer_share_selection(self):
        summary, observations, profile = self.detected()
        source = build_source(summary,observations,profile,'new-observation',asset_set=self.selected)
        scene = source['scene']
        self.assertEqual(scene['machine_asset'],self.selected['machine']['native_asset'])
        self.assertEqual(scene['asset_set'],self.selected)
        _validate_scene(scene)
        episode = SimpleNamespace(machine_asset=scene['machine_asset'],metadata=episode_metadata(self.selected))
        self.assertEqual(selected_cup_asset(self.root,episode),self.selected['cup']['native_asset'])
        episode.metadata = {}
        with self.assertRaisesRegex(ValueError,'episode and prepared'):
            selected_cup_asset(self.root,episode)
        check_geometry(self.selected,'a'*64,'b'*64)
        with self.assertRaisesRegex(ValueError,'native cup geometry differs'):
            check_geometry(self.selected,'a'*64,'f'*64)

    def test_mixed_detector_and_old_installation_are_rejected(self):
        summary, observations, profile = self.detected()
        summary['runtime_config_id'] = 'f'*64
        with self.assertRaisesRegex(ValueError,'detection runtime differs'):
            build_source(summary,observations,profile,'wrong-runtime',asset_set=self.selected)
        summary, observations, profile = self.detected()
        observations[0]['tracks'][1]['provenance']['asset']['asset_id'] = 'old-cup'
        with self.assertRaisesRegex(ValueError,'detected cup model differs'):
            build_source(summary,observations,profile,'wrong-cup',asset_set=self.selected)
        summary, observations, profile = self.detected()
        profile['scene_support'] = dict(machine_asset='objects/owner_coffee_machine_00003/v1',
            machine_root_z_m=.78,cup_root_z_m=.78,source='old installation geometry')
        with self.assertRaisesRegex(ValueError,'installation support belongs to another machine'):
            build_source(summary,observations,profile,'wrong-support',asset_set=self.selected)

    def test_suite_keeps_asset_metadata_and_does_not_relabel_legacy(self):
        summary, observations, profile = self.detected()
        detection = self.root/'detection'
        detection.mkdir()
        (detection/'summary.json').write_text(json.dumps(summary))
        (detection/'observations.jsonl').write_text('\n'.join(map(json.dumps,observations))+'\n')
        path = self.root/'profile.json'
        path.write_text(json.dumps(profile))
        output = self.root/'bridge'
        convert(path,detection,output,'new',assets=self.root,asset_set_digest=self.selected['asset_set_digest'])
        episode = json.loads((output/'suite.json').read_text())['episodes'][0]
        self.assertEqual(episode['metadata']['scene_asset_set'],self.selected)
        bad = deepcopy(self.selected)
        bad['cup']['native_asset'] = 'objects/../../old'
        with self.assertRaisesRegex(ValueError,'versioned objects'):
            validate(seal(bad))

    def test_new_cup_requires_its_own_seed_and_never_uses_legacy_gap(self):
        from gripper_distance_plan import requested_metal_gap_mm
        summary,observations,profile=self.detected()
        source=build_source(summary,observations,profile,'missing-grasp',asset_set=self.selected)
        with self.assertRaisesRegex(ValueError,'legacy 65mm fallback is forbidden'):
            requested_metal_gap_mm(source)
        cup=SimpleNamespace(candidate_grasp_band_m=(.035,.1),profile_sha256='b'*64,
            required_opening_m=lambda low,high:.070)
        command=derive_grasp_command(cup)
        self.assertEqual(command['metal_gap_mm'],71.)
        self.assertFalse(command['derivation']['qualified'])
        self.selected['cup']['grasp_command']=command;seal(self.selected)
        source=build_source(summary,observations,profile,'selected-grasp',asset_set=self.selected)
        self.assertEqual(requested_metal_gap_mm(source),71.)
        source['grasp_command']['metal_gap_mm']=65.
        with self.assertRaisesRegex(ValueError,'matching geometry-bound grasp'):
            requested_metal_gap_mm(source)


if __name__=='__main__':
    unittest.main()
