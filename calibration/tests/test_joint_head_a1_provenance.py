"""An A1 switch must not silently restore or reuse X5 calibration parameters."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from joint_head_workbench import Workbench


class A1ProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.app = Workbench.__new__(Workbench)
        self.app.output = Path(self.temp.name)
        self.app.config = {'asset_geometry_id': 'a1-r2', 'asset_seal_sha256': 'seal-r2'}
        self.app.active_environment = lambda: 'hardware'
        self.app.robot_source = lambda: 'physical'
        self.app.calibration = None
        self.report = dict(camera_T_world_optical=np.eye(4).tolist(),
            joint_offsets_rad={s: [0.] * 6 for s in ('left', 'right')},
            T_link6_marker={str(i): np.eye(4).tolist() for i in (24, 30)},
            robot_source='physical', simulation_only=False,
            asset_geometry_id='a1-r2', asset_seal_sha256='seal-r2')

    def row(self, report):
        candidate = self.app.calibration_candidate('joint_candidate', 'capture', report)
        return dict(id=candidate['id'], batch='capture', candidate='joint_candidate',
                    environment='hardware', robot_source='physical', simulation_only=False,
                    parameters=candidate)

    def test_candidate_identity_and_snapshot_retain_asset(self):
        current = self.row(self.report)
        old = self.row({**self.report, 'asset_geometry_id': 'x5'})
        self.assertNotEqual(current['id'], old['id'])
        snapshot = Workbench.calibration_snapshot(current['parameters'])
        self.assertEqual(snapshot['asset_geometry_id'], 'a1-r2')
        self.assertEqual(snapshot['asset_seal_sha256'], 'seal-r2')
        self.assertTrue(snapshot['snapshot_complete'])

    def test_choices_and_install_entry_guard_reject_foreign_and_legacy(self):
        current = self.row(self.report)
        foreign = self.row({**self.report, 'asset_geometry_id': 'x5'})
        legacy = self.row({k: v for k, v in self.report.items() if not k.startswith('asset_')})
        self.assertEqual(self.app.environment_results([foreign, current, legacy]), [current])
        self.app.require_calibration_environment(current)
        for row in (foreign, legacy):
            with self.assertRaisesRegex(ValueError, 'A1'):
                self.app.require_calibration_environment(row)

    def test_restore_does_not_reactivate_legacy_parameters(self):
        for asset in ('x5', None):
            report = {**self.report, 'asset_geometry_id': asset}
            row = self.row(report)
            (self.app.output / 'realtime-calibration.json').write_text(json.dumps(row))
            self.app.restore_calibration()
            self.assertIsNone(self.app.calibration)

    def test_restore_keeps_matching_parameters(self):
        row = self.row(self.report)
        (self.app.output / 'realtime-calibration.json').write_text(json.dumps(row))
        self.app.restore_calibration()
        self.assertEqual(self.app.calibration['parameters']['asset_geometry_id'], 'a1-r2')

    def test_mode_restore_drops_foreign_geometry_selection(self):
        row = self.row({**self.report, 'asset_geometry_id': 'x5'})
        self.app.select_environment({'calibration': row})
        self.assertIsNone(self.app.calibration)

    def test_historical_head_overlay_rejects_foreign_geometry(self):
        row = self.row({**self.report, 'asset_geometry_id': 'x5'})
        self.app.review_batch = lambda _: dict(frames=[dict(image='frame.png', q_reported_rad=[0.] * 12)],
                                             results=[{**row, 'kind': 'head'}])
        self.app.calibration_results = lambda: [row]
        with self.assertRaisesRegex(ValueError, '几何|geometry'):
            self.app.historical_overlay(dict(batch_id='capture', frame_index=0, result_id=row['id']))

    def test_live_dataset_retains_asset_identity(self):
        from types import SimpleNamespace
        from examples.scripts.debug.joint_head_workbench_live import Session
        session = Session(SimpleNamespace(velocity_limits=np.ones(12), limits=np.tile([-3., 3.], (12, 1)), k={}), self.app.output,
                          self.app.config)
        session.batch_dir = self.app.output
        session.capture_id = 'capture'
        session.camera = {}
        session.rows = []
        session.planned_frames = []
        session.finish()
        data = json.loads((self.app.output / 'dataset.json').read_text())
        for key in ('asset_geometry_id', 'asset_seal_sha256'):
            self.assertEqual(data[key], self.app.config[key])
