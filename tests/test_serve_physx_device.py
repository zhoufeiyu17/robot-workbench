"""Startup input/configuration checks only; importing the CLI must not load Isaac."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from copy import deepcopy
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import serve_physx_device
from serve_physx_device import configure_calibration,nominal_inputs,printed_marker_fixture,plant_joint_offsets
from robot_workbench.physx_camera import camera_intrinsics,CAMERAS
from robot_workbench.sample_clock import clock_spec


class ResidentConfigurationTest(unittest.TestCase):
    def test_invalid_plant_offsets_fail_before_inputs_output_or_gpu(self):
        invalid=(None,{},'0',[],[0.]*11,[0.]*13,[True]+[0.]*11,['0']+[0.]*11,
                 [float('nan')]+[0.]*11,[float('inf')]+[0.]*11,[-float('inf')]+[0.]*11,[10**400]+[0.]*11)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base.json';output=root/'output'
            argv=['serve','--base',str(base),'--nominal-config',str(root/'nominal.json'),
                  '--output',str(output),'--gpu','1']
            for value in invalid:
                base.write_text(json.dumps(dict(plant_joint_offsets_rad=value)))
                for dry in (False,True):
                    with self.subTest(value=value,dry_run=dry),patch.object(sys,'argv',argv+(['--dry-run'] if dry else [])), \
                            patch.object(serve_physx_device,'nominal_inputs') as nominal,patch.object(serve_physx_device,'run') as run:
                        with self.assertRaisesRegex(ValueError,'twelve finite'):serve_physx_device.main()
                        nominal.assert_not_called();run.assert_not_called();self.assertFalse(output.exists())
            self.assertNotIn('isaaclab.app',sys.modules)

    def test_default_and_explicit_zero_preserve_existing_backend_sdk_mapping(self):
        from test_physx_boundary import LaggingPlant
        from robot_workbench.physx import PhysxBackend
        from arx_env.joint_head_jaw_mapping import command_sdk
        absent={};explicit=dict(plant_joint_offsets_rad=[0.]*12)
        before=deepcopy((absent,explicit))
        values=([0.]*12,plant_joint_offsets(absent),plant_joint_offsets(explicit))
        backends=[PhysxBackend(LaggingPlant(),joint_offsets=value,metal_gap_offset_mm=.008) for value in values]
        target=[.05]*14;target[6]=command_sdk('left',80.);target[13]=command_sdk('right',80.)
        for backend in backends:
            for side in ('left','right'):backend.send(side,target)
            backend.advance(1/240)
        for backend in backends[1:]:
            for key in ('qpos','qvel'):self.assertEqual(backend.read_calibration_state()[key],backends[0].read_calibration_state()[key])
            self.assertEqual(backend.plant.read(),backends[0].plant.read())
            self.assertEqual(backend.get_bimanual_joint_position_limits(),backends[0].get_bimanual_joint_position_limits())
            self.assertEqual(backend.gap_offset,.008)
        self.assertEqual((absent,explicit),before)

    def test_nonzero_truth_applies_once_in_original_backend_command_and_feedback(self):
        from test_physx_boundary import LaggingPlant
        from robot_workbench.physx import PhysxBackend
        from arx_env.joint_head_jaw_mapping import command_sdk
        values=[.003,0.,0.,0.,0.,0.,-.003,0.,0.,0.,0.,0.]
        base=dict(plant_joint_offsets_rad=values);before=deepcopy(base)
        plant=LaggingPlant();backend=PhysxBackend(plant,joint_offsets=plant_joint_offsets(base),metal_gap_offset_mm=.008)
        target=[.02]*14;target[6]=command_sdk('left',80.);target[13]=command_sdk('right',80.)
        for n,side in enumerate(('left','right')):
            initial=backend.read_calibration_state()['qpos']
            for j in range(6):self.assertAlmostEqual(initial[n*7+j],-values[n*6+j])
            backend.send(side,target)
            native_target=backend.pending[side][0]
            for j in range(6):self.assertAlmostEqual(native_target[j],target[n*7+j]+values[n*6+j])
            self.assertAlmostEqual(native_target[6],80.-.008)
        backend.advance(1/240)
        feedback=backend.read_calibration_state()
        for n,side in enumerate(('left','right')):
            for j in range(6):
                self.assertAlmostEqual(feedback['qpos'][n*7+j],plant.q[side][j]-values[n*6+j])
                self.assertEqual(feedback['qvel'][n*7+j],plant.v[side][j])
        self.assertEqual(base,before)

    def test_printed_face_is_outside_housing_without_changing_estimator_mount(self):
        candidate={'T_link6_marker':{'24':[[0,0,1,.1],[1,0,0,.2],[0,1,0,.3],[0,0,0,1]],
                                     '30':[[1,0,0,.4],[0,1,0,.5],[0,0,1,.6],[0,0,0,1]]}}
        before=deepcopy(candidate);markers=printed_marker_fixture(candidate)
        self.assertEqual(candidate,before)
        self.assertAlmostEqual(markers[0]['T_link_marker'][0][3],.0999)
        self.assertAlmostEqual(markers[1]['T_link_marker'][2][3],.5999)
        for marker in markers:
            original=candidate['T_link6_marker'][str(marker['id'])]
            for axis in range(3):self.assertEqual(marker['T_link_marker'][axis][:3],original[axis][:3])
            self.assertEqual(marker['print_surface_offset_m'],.0001)

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
            self.assertNotIn('plant_joint_offsets_rad',result)
            self.assertTrue(Path(result['head_motion_seed']).is_file())
            (assets/'robot.urdf').write_text('changed')
            with self.assertRaisesRegex(ValueError,'differs'):nominal_inputs(config)


if __name__=='__main__':unittest.main()
