"""Alignment contract tests require no LeRobot, GPU, SDK, or network."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from robot_workbench.lerobot_export import alignment, vector, verified_export, digest, CAMERAS


def state(value, jaw=65.):
    return dict(schema_version='physical14.v1', encoding='absolute', units=dict(joints='rad', grippers='mm'),
        left_arm_joints=[value] * 6, left_gripper=jaw, right_arm_joints=[-value] * 6, right_gripper=None)


class ExportAlignmentTest(unittest.TestCase):
    def test_physical_device_connection_receives_the_save_policy_in_both_topologies(self):
        from robot_workbench.collection_setup import connection_config
        setup=dict(environment='physical',robot_id='arx6',control_url='http://127.0.0.1:1234',token_file='/fixture/token',
            trusted_binding_path='/fixture/binding',local_recording_root='/fixture/raw',resident_root='/fixture',
            camera_serials=dict(head='h',left='l',right='r'),producer_ports=dict(head=1200,left=1201,right=1202),
            producer_directories=dict(base_0='/fixture/h',left_wrist_0='/fixture/l',right_wrist_0='/fixture/r'))
        for endpoint in (None,dict(host='cpu-fixture',port=739)):
            result=connection_config(setup,'/fixture/pipeline','/fixture/calibration','/fixture/capture','/fixture/jobs',endpoint=endpoint)
            self.assertEqual(result['dataset_format'],'lerobot-v2.1')
        setup['environment']='fake'
        result=connection_config(setup,'/fixture/pipeline','/fixture/calibration','/fixture/capture','/fixture/jobs')
        self.assertNotIn('dataset_format',result)

    def inputs(self):
        commands = [dict(send_wall_ns=1_000_000_000 + i * 100_000_000, target=state(i, 60.)) for i in range(3)]
        feedback = [dict(sample_wall_ns=990_000_000 + i * 20_000_000, state=state(i / 100)) for i in range(12)]
        cameras = {key: [dict(host_receive_ns=1_000_000_000 + i * 100_000_000, frame_number=i + 100)
            for i in range(3)] for key in CAMERAS}
        return commands, feedback, cameras

    def test_reference_feedback_are_distinct_and_original_times_are_preserved(self):
        args = self.inputs()
        rows = list(alignment(*args, fps=10, disabled=['right']))
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[1]['action'][0], 1.)
        self.assertAlmostEqual(rows[1]['state'][0], .055)
        self.assertEqual(rows[1]['feedback_wall_ns'], [1_090_000_000, 1_110_000_000])
        self.assertEqual(rows[1]['cameras']['base_0']['frame_number'], 101)
        self.assertFalse(rows[1]['state_valid'][13])
        self.assertFalse(rows[1]['action_valid'][13])
        self.assertEqual(rows[1]['state'][13], 0.)

    def test_missing_enabled_feedback_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'missing enabled'):
            vector(state(0.))

    def test_camera_and_feedback_gaps_are_rejected(self):
        commands, feedback, cameras = self.inputs()
        bad = deepcopy(cameras)
        bad['base_0'] = bad['base_0'][:1]
        with self.assertRaisesRegex(ValueError, 'no captured frame'):
            list(alignment(commands, feedback, bad, fps=10, disabled=['right']))
        with self.assertRaisesRegex(ValueError, 'feedback gap'):
            list(alignment(commands, [feedback[0], feedback[-1]], cameras, fps=10, disabled=['right']))

    def test_partial_interval_and_timestamp_regression_are_rejected(self):
        commands, feedback, cameras = self.inputs()
        with self.assertRaisesRegex(ValueError, 'complete command interval'):
            list(alignment(commands, feedback[1:], cameras, fps=10, disabled=['right']))
        cameras['left_wrist_0'][1]['host_receive_ns'] = cameras['left_wrist_0'][0]['host_receive_ns']
        with self.assertRaisesRegex(ValueError, 'advancing'):
            list(alignment(commands, feedback, cameras, fps=10, disabled=['right']))

    def test_reuse_requires_the_same_raw_and_unchanged_dataset_files(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp);raw=root/'raw';raw.mkdir();(raw/'manifest.json').write_text('{}')
            data=root/'dataset';(data/'audit').mkdir(parents=True);(data/'table').write_text('sealed')
            receipt=dict(schema='workbench.lerobot_export.v1',status='NATIVE_READBACK_PASS',
                episodes=[dict(raw_manifest_sha256=digest(raw/'manifest.json'))],
                files={'table':dict(bytes=6,sha256=digest(data/'table'))})
            (data/'audit/export-receipt.json').write_text(json.dumps(receipt))
            self.assertEqual(verified_export(data,[raw]),receipt)
            with self.assertRaisesRegex(ValueError,'differs from this raw'):
                verified_export(data,[raw],fps=50)
            (data/'table').write_text('broken')
            with self.assertRaisesRegex(ValueError,'file changed'):
                verified_export(data,[raw])
            (raw/'manifest.json').write_text('{"other":true}')
            with self.assertRaisesRegex(ValueError,'differs from this raw'):
                verified_export(data,[raw])

    def test_maintenance_return_cannot_be_exported_as_a_new_training_episode(self):
        from robot_workbench.lerobot_export import export
        with TemporaryDirectory() as tmp:
            root=Path(tmp);raw=root/'raw';raw.mkdir()
            for source in (dict(schema_version='physical_cup_maintenance_source.v1'),
                           dict(schema_version='physical_collection_source.v1',training_observation=False)):
                with patch('real_recording.verify_raw',return_value=({},source,None)):
                    with self.assertRaisesRegex(ValueError,'maintenance return is excluded'):
                        export([raw],root/'dataset',repo_id='workbench/cpu-fixture',task='CPU only')
            self.assertFalse((root/'dataset').exists())


if __name__ == '__main__':
    unittest.main()
