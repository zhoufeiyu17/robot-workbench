"""Retired CPU inference graphs must release without invoking cyclic GC."""
from pathlib import Path
import gc,sys,weakref
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
import test_inference_owner as owner_fixture
import test_inference_workflow as workflow_fixture

class InferenceMemoryTests(unittest.TestCase):
    def test_unstarted_runner_and_loop_release_without_cyclic_gc(self):
        fixture=owner_fixture.InferenceOwnerTests('runTest');fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.configure(duration_s=2);fixture.runtime.owner.execution_instance_id='cpu-memory-check'
        config=fixture.runtime.inference_configuration;proxy=SimpleNamespace(joint_limits=fixture.runtime.owner.cache['joint_limits'])
        enabled=gc.isenabled();self.addCleanup(gc.enable if enabled else gc.disable);gc.collect();gc.disable()
        refs=[]
        for _ in range(100):
            runner=fixture.factory(fixture.runtime.owner,proxy,fixture.runtime.owner.cache['calibration_feedback'],config)
            refs.extend((weakref.ref(runner),weakref.ref(runner.loop)))
            fixture.instances.clear();del runner
        self.assertFalse(any(ref()is not None for ref in refs))

    def test_retired_workflow_run_releases_callbacks_without_cyclic_gc(self):
        fixture=workflow_fixture.InferenceWorkflowTests('runTest');fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.prepare();fixture.workflow.preview_episode(seed=0,duration_s=2)
        config,factory=fixture.configured[-1];fixture.owner.environment='fake';fixture.owner.shared_target_profile=None
        enabled=gc.isenabled();self.addCleanup(gc.enable if enabled else gc.disable);gc.collect();gc.disable()
        runner=factory(fixture.owner,SimpleNamespace(joint_limits=fixture.limits),fixture.feedback,config)
        fixture.owner.runner=runner
        refs=(weakref.ref(runner),weakref.ref(runner.loop))
        fixture.workflow.current=None;fixture.owner.runner=None;fixture.configured.clear();del runner
        self.assertFalse(any(ref()is not None for ref in refs))

    def test_recording_admission_callbacks_do_not_retain_retired_runs(self):
        fixture=workflow_fixture.InferenceWorkflowTests('runTest');fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.prepare();fixture.workflow.preview_episode(seed=0,duration_s=2,recording_enabled=True)
        config,factory=fixture.configured[-1];fixture.owner.environment='fake';fixture.owner.shared_target_profile=None
        enabled=gc.isenabled();self.addCleanup(gc.enable if enabled else gc.disable);gc.collect();gc.disable()
        runner=factory(fixture.owner,SimpleNamespace(joint_limits=fixture.limits),fixture.feedback,config)
        fixture.owner.runner=runner
        payload=dict(clock=config['clock'],execution_owner=dict(owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner'),
            preparation_deadline_monotonic=time.monotonic()+1)
        with patch('robot_workbench.recording_process.ProcessRecordingSession',workflow_fixture.Recorder):runner.record('prepare',payload)
        run=fixture.workflow.current;recorder=run['recorder']
        self.assertTrue(runner.record_prepared())
        refs=[weakref.ref(value)for value in (runner,runner.loop,run,recorder)]
        fixture.workflow.current=None;fixture.owner.runner=None;fixture.owner.recording=None;fixture.configured.clear()
        del run,recorder,runner
        self.assertFalse(any(ref()is not None for ref in refs))

if __name__=='__main__':unittest.main()
