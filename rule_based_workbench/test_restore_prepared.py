"""Explicit frozen-plan recovery with real validators and CPU-only artifacts."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from playground import Playground
from test_initial_state import owner
import test_playground_assisted as assisted_fixture


class RestorePreparedTest(unittest.TestCase):
    def setUp(self):
        self.fixture=assisted_fixture.AssistedEntryTest()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.plan=self.fixture.plan();self.bridge=self.fixture.fixture.bridge
        self.bridge.config['playground']={'enabled':True,**self.fixture.config}
        self.current=deepcopy(self.fixture.source['initial_state']['measured_reported'])
        self.disabled=[];self.connected=True
        def current_owner():
            row=owner(deepcopy(self.current),stamp=time.time_ns(),epoch=91)
            # A deliberate SDK/service restart; old PID/epoch cannot be reused
            # as execution authority, but a new explicit review can be issued.
            row.update(pid=56789,holder='collection',connected=self.connected,
                pedal=dict(input_healthy=True,physically_down=False))
            row['calibration_feedback'].update(gripper_feedback_valid=[side not in self.disabled for side in ('left','right')],
                                               can_output_guard={'disabled_grippers':list(self.disabled)})
            return row
        self.bridge.owner=current_owner
        self.bridge.status=lambda:dict(connected=self.connected,owner=current_owner(),
            cameras={name:{'ready':True} for name in ('base_0','left_wrist_0','right_wrist_0')})
        self.bridge.control=Mock(side_effect=AssertionError('restore cannot send control commands'))
        self.recover=Mock(side_effect=AssertionError('restore cannot start GPU services'))
        self.service=Playground(self.fixture.directory,self.bridge,
            lambda:dict(status='stopped',planner={'status':'stopped','model_initialized':False}),recover_planner=self.recover)
        Path(self.plan['first_frame_path']).write_bytes(b'\xff\xd8CPU-only image\xff\xd9')
        self.service.checkpoint(self.plan)

    def test_new_sdk_pid_restores_reviewable_plan_without_warming_or_reobserving(self):
        with patch('playground_pipeline.prepare_environment',side_effect=AssertionError('no preparation')),\
                patch('playground_pipeline.warmup',side_effect=AssertionError('no observations')),\
                patch('playground_pipeline.plan',side_effect=AssertionError('no planning')),\
                patch('playground_pipeline.execute',side_effect=AssertionError('no execution')):
            restored=self.service.restore_prepared()
        state=self.service.snapshot()
        self.assertTrue(restored['validated']);self.assertTrue(state['can_execute'])
        self.assertEqual(state['plan']['timeline_digest'],self.plan['timeline_digest'])
        self.assertEqual(state['warmup']['status'],'not_started')
        self.assertNotEqual(state['warmup'].get('model_initialized'),True)
        self.assertFalse(state['environment']['ready'])  # GPU warm status remains truthful.
        self.assertTrue(self.service.state['environment']['ready'])  # Frozen-file readiness only.
        self.assertEqual(state['environment']['scope'],'frozen_plan_and_current_owner_only')
        self.assertEqual(self.service.media('frame',Path(restored['output']).name),Path(restored['first_frame_path']))
        self.assertIsNone(self.service.thread)
        self.bridge.control.assert_not_called();self.recover.assert_not_called()

    def test_drift_disabled_change_or_disconnection_reject_without_fallback(self):
        for failure in ('drift','disabled','disconnected'):
            self.current=deepcopy(self.fixture.source['initial_state']['measured_reported'])
            self.disabled=[];self.connected=True
            if failure=='drift':self.current['right_arm_joints'][0]+=.011
            elif failure=='disabled':self.disabled=['right']
            else:self.connected=False
            with self.subTest(failure=failure),self.assertRaises(ValueError):self.service.restore_prepared()
            self.assertFalse(self.service.snapshot()['can_execute'])
            self.assertFalse(self.service.state['plan']['validated'])
        self.bridge.control.assert_not_called();self.recover.assert_not_called()

    def test_external_artifact_and_corrupt_first_frame_are_rejected(self):
        outside=self.fixture.fixture.root/'outside.json'
        outside.write_bytes(Path(self.plan['source_path']).read_bytes())
        self.service.checkpoint(dict(self.plan,source_path=str(outside)))
        with self.assertRaisesRegex(ValueError,'任务目录'):self.service.restore_prepared()
        self.service.checkpoint(self.plan)
        Path(self.plan['first_frame_path']).write_bytes(b'not an image')
        with self.assertRaisesRegex(ValueError,'PNG/JPEG'):self.service.restore_prepared()
        self.bridge.control.assert_not_called();self.recover.assert_not_called()

    def test_unknown_interrupted_execution_cannot_become_reviewable(self):
        execution=self.service.directory/'jobs/interrupted';execution.mkdir()
        self.service.checkpoint(self.plan,execution_pending=True,execution_output=execution)
        with self.assertRaisesRegex(ValueError,'未执行冻结计划'):self.service.restore_prepared()
        self.assertFalse(self.service.snapshot()['can_execute'])
        saved=json.loads((self.service.directory/'prepared-plan.json').read_text())
        self.assertTrue(saved['execution_pending'])
        self.bridge.control.assert_not_called();self.recover.assert_not_called()


if __name__=='__main__':unittest.main()
