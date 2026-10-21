"""CPU one-off planning entry: real source/transport/contracts, fake GPU only."""
from copy import deepcopy
import json
from pathlib import Path
import shlex
import sys
import time
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
import playground_pipeline as pipeline
from integration import _digest
from real_contract import validate_assisted_cup_geometry
from run_pipeline import load_config
from source_from_detection import build_assisted_source
from test_initial_state import initial_fixture,owner
from test_operator_assisted_source import receipt_fixture
import test_playground_motiongen_entry as entry_fixture


class AssistedEntryTest(unittest.TestCase):
    def setUp(self):
        # Reuse the established local asset/contract and synthetic-GPU fixture.
        self.fixture=entry_fixture.PlaygroundMotionGenEntryTest()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        f=self.fixture;profile=f.source['calibration_profile']
        initial=initial_fixture(profile)
        accepted=dict(f.binding,initial_state=initial,calibration_profile_digest=_digest(profile))
        receipt,_,_=receipt_fixture()
        receipt.update(robot_id=accepted['robot_id'],camera_serials=accepted['camera_serials'],
            calibration_profile_digest=_digest(profile),initial_state_digest=_digest(initial))
        self.source,self.suite,self.binding=build_assisted_source(receipt,profile,'assisted-cpu-source',accepted)
        self.inputs=f.root/'assisted';self.inputs.mkdir()
        for name,value in (('source.json',self.source),('suite.json',self.suite),('hardware-binding.json',self.binding)):
            f.write(self.inputs/name,value)
        f.write(f.root/'binding.json',accepted)
        f.bridge.config['trusted_binding_path']=str(f.root/'binding.json')
        f.bridge.camera=Mock(side_effect=AssertionError('assisted input must not recapture or redetect'))
        f.bridge.command=Mock(side_effect=AssertionError('planning must not acquire control or move'))
        f.bridge.owner=lambda:owner(deepcopy(initial['measured_reported']),stamp=time.time_ns())
        self.directory=f.root/'playground';self.output=self.directory/'jobs/assisted'
        self.config=dict(pipeline_config=str(f.root/'config.json'))

    def run_remote(self,stages,output,contract,*args):
        f=self.fixture;f.prepared={'pipeline_config':str(Path(output)/'config.json')}
        cfg,_,_=load_config(Path(f.prepared['pipeline_config']))
        for stage in stages:
            name=stage['name'];f.stages.append(name)
            if name.startswith('preflight-'):
                from a1_asset_validation import sha256
                for path,digest in contract['provenance']['input_sha256'].items():
                    self.assertEqual(sha256(path),digest)
            elif name=='initialize':f.remote.mkdir()
            elif name=='shared-storage':self.assertTrue(f.remote.is_dir())
            elif name=='operator-assisted-source-import':
                from run_pipeline import bounded_command
                bounded_command(stage,600)
                tokens=shlex.split(stage['argv'][-1])
                if tokens[:4]==['timeout','-k','30','60']:tokens=shlex.split(tokens[-1])
                index=tokens.index('-c')
                script=tokens[index+1];arguments=tokens[index+2:]
                arguments[3]=str(f.remote/'detection-bridge')
                with patch.object(sys,'argv',['-c',*arguments]):
                    exec(compile(script,'<CPU assisted import>','exec'),{'__name__':'cpu_assisted_import'})
            elif name=='playground-geometry-inputs':
                imported=json.loads((f.remote/'detection-bridge/source.json').read_text())
                cup=imported['cup_geometry']
                actual=dict(profile_sha256=cup['profile_sha256'],bands=cup['bands'])
                if f.tamper=='cup-size':actual=deepcopy(actual);actual['bands'][0]['z_max_m']=.13
                validate_assisted_cup_geometry(imported,actual)
                f.write(f.remote/'playground-scene/planning-input.json',{'cup':actual})
                (f.remote/'playground-scene/first-frame.png').write_bytes(b'CPU render')
            elif name=='playground-plan':f.synthetic_gpu_result()
            else:self.fail('unexpected automatic-detection/rollout stage: '+name)

    def plan(self,**kwargs):
        with patch.object(pipeline,'run_stages',self.run_remote),patch.object(pipeline,'fetch',self.fixture.fetch):
            return pipeline.plan_assisted(self.config,self.output,self.fixture.bridge,.3,self.inputs,**kwargs)

    def test_full_entry_uses_only_explicit_inputs_and_existing_checkpoint(self):
        result=self.plan();f=self.fixture
        self.assertEqual(f.stages,['preflight-simulated-real','preflight-dsw-scene','initialize','shared-storage',
            'operator-assisted-source-import','playground-geometry-inputs','playground-plan'])
        self.assertTrue(result['validated'])
        self.assertEqual(result['observation_mode'],'operator_assisted_rgbd')
        self.assertEqual(json.loads(Path(result['source_path']).read_text()),self.source)
        self.assertNotIn('detection',self.source['scene']['source'])
        f.bridge.camera.assert_not_called();f.bridge.command.assert_not_called()
        f.write(self.directory/'prepared-plan.json',dict(schema_version='playground_prepared_plan.v1',plan=result))
        self.assertEqual(pipeline.restore_plan(self.config,self.directory,f.bridge),result)
        cfg,contract,_=load_config(Path(result['config_path']))
        self.assertNotEqual(cfg['remote_run_root'],f.cfg['remote_run_root'])
        self.assertFalse(cfg['render_replay'])
        self.assertEqual(len([p for p in contract['provenance']['input_sha256'] if '/assisted-inputs-' in p]),3)

    def test_stale_owner_or_drift_rejected_before_any_upload(self):
        f=self.fixture
        for change in ('pid','epoch','joint','disabled'):
            current=owner(deepcopy(self.source['initial_state']['measured_reported']),stamp=time.time_ns())
            if change=='pid':current['pid']+=1
            elif change=='epoch':current['command_epoch']+=1
            elif change=='joint':current['physical_feedback']['right_arm_joints'][0]+=.011
            else:current['calibration_feedback'].update(gripper_feedback_valid=[True,False],can_output_guard={'disabled_grippers':['right']})
            f.bridge.owner=lambda value=current:value
            with self.subTest(change=change),patch('artifact_transport.transfer') as transfer:
                with self.assertRaisesRegex(ValueError,'fresh measured state'):self.plan()
                transfer.assert_not_called()

    def test_changed_suite_and_missing_assistance_never_use_automatic_fallback(self):
        f=self.fixture
        bad=deepcopy(self.suite);bad['episodes'][0]['environment_from_cup']['position'][0]+=.01
        f.write(self.inputs/'suite.json',bad)
        with patch('artifact_transport.transfer') as transfer:
            with self.assertRaisesRegex(ValueError,'assisted suite'):self.plan()
            transfer.assert_not_called()
        f.write(self.inputs/'suite.json',self.suite)
        bad=deepcopy(self.source);bad['adapter']='foundationpose';f.write(self.inputs/'source.json',bad)
        with patch('artifact_transport.transfer') as transfer:
            with self.assertRaisesRegex(ValueError,'explicitly accepted'):self.plan()
            transfer.assert_not_called()
        f.bridge.camera.assert_not_called()

    def test_wrong_actual_cup_stops_before_planning_and_existing_run_is_not_overwritten(self):
        f=self.fixture;f.tamper='cup-size'
        with self.assertRaisesRegex(ValueError,'actual planner cup'):self.plan()
        self.assertNotIn('playground-plan',f.stages)
        f.remote.joinpath('keep.txt').write_text('original')
        f.tamper=None;self.output=self.directory/'jobs/second'
        with self.assertRaises(FileExistsError):self.plan()
        self.assertEqual((f.remote/'keep.txt').read_text(),'original')

    def test_final_receipt_keeps_this_exact_assisted_request(self):
        finish=pipeline._finish_plan
        def replaced(*args,**kwargs):
            result=finish(*args,**kwargs)
            source=json.loads(Path(result['source_path']).read_text())
            source['scene']['source']['assistance_receipt']['request_id']='another-accepted-request'
            Path(result['source_path']).write_text(json.dumps(source))
            return result
        with patch.object(pipeline,'_finish_plan',side_effect=replaced):
            with self.assertRaisesRegex(ValueError,'this explicit assisted request'):self.plan()

    def test_cancellation_prevents_upload_and_owner_check(self):
        self.fixture.bridge.owner=Mock(side_effect=AssertionError('cancelled'))
        with patch('artifact_transport.transfer') as transfer:
            with self.assertRaises(InterruptedError):self.plan(cancel=lambda:True)
            transfer.assert_not_called()


if __name__=='__main__':unittest.main()
