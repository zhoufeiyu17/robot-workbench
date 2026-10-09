"""Startup input/configuration checks only; importing the CLI must not load Isaac."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from serve_physx_device import configure_calibration,nominal_inputs
from robot_workbench.physx_camera import camera_intrinsics,CAMERAS
from robot_workbench.sample_clock import clock_spec


class ResidentConfigurationTest(unittest.TestCase):
    def test_nominal_files_are_sealed_and_generated_calibration_root_is_empty(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);assets=root/'nominal';assets.mkdir();output=root/'run';output.mkdir()
            files={'robot.urdf':'<robot name="fixture"/>','manifest.json':'{}','scene_seed.json':'{}',
                   'candidate.json':'{}','camera_seed.json':'{}','jaw-mapping.json':'{}'}
            for name,value in files.items():(assets/name).write_text(value)
            seal={name:hashlib.sha256((assets/name).read_bytes()).hexdigest() for name in files}
            (assets/'SHA256SUMS.json').write_text(json.dumps(seal))
            nominal=dict(model_family='ac_one',simulation_only=True,asset_geometry_id='g',
                asset_seal_sha256=hashlib.sha256((assets/'SHA256SUMS.json').read_bytes()).hexdigest(),
                **{key:str(assets/name) for key,name in dict(urdf='robot.urdf',mesh_manifest='manifest.json',
                    scene_seed='scene_seed.json',candidate='candidate.json',camera_seed='camera_seed.json',
                    jaw_mapping_file='jaw-mapping.json').items()})
            config=root/'nominal.json';config.write_text(json.dumps(nominal))
            self.assertEqual(nominal_inputs(config),nominal)
            rig={'device_calibrations':{name:dict(serial=str(i),width=4,height=3,
                 intrinsics=dict(fx=100.,fy=100.,cx=2.,cy=1.),distortion=dict(coefficients=[0.]*5))
                 for i,name in enumerate(CAMERAS.values())}}
            clock=clock_spec('fake',source_clock_id='resident-fixture')
            result=configure_calibration(nominal,output,camera_intrinsics(rig),dict(head=18001,left=18002,right=18003),
                18004,root/'credential-path',source_clock=clock)
            self.assertEqual(result['environment'],'fake')
            self.assertEqual(result['robot_source'],'simulated')
            self.assertEqual(result['clock'],clock)
            self.assertEqual(list(Path(result['calibration_root']).iterdir()),[])
            self.assertNotIn('live_backend',result)
            self.assertNotIn('truth_fixture',result)
            self.assertTrue(Path(result['head_motion_seed']).is_file())
            (assets/'robot.urdf').write_text('changed')
            with self.assertRaisesRegex(ValueError,'differs'):nominal_inputs(config)


if __name__=='__main__':unittest.main()
