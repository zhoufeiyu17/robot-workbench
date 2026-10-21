"""CPU live Playground state/checkpoint installation; no network or controller."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from playground import Playground
from source_from_detection import build_assisted_source
from test_initial_state import owner
from test_operator_assisted_source import receipt_fixture


class AssistedUITest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        receipt,profile,binding=receipt_fixture()
        self.source,self.suite,self.binding=build_assisted_source(receipt,profile,'ui-assisted',binding)
        self.request_id=receipt['request_id'];self.inputs=self.root/'inputs';self.inputs.mkdir()
        for name,value in (('source.json',self.source),('suite.json',self.suite),('hardware-binding.json',self.binding)):
            (self.inputs/name).write_text(json.dumps(value))
        self.connected=True;self.commands=[]
        self.bridge=SimpleNamespace(config=dict(environment='physical',playground=dict(enabled=True,
            operator_assisted_directory=str(self.inputs))),owner=self.owner,status=self.health,
            control=lambda command:self.commands.append(command))
        self.initial=dict(status='stopped',planner=dict(status='stopped',model_initialized=False),
                          scene=dict(status='stopped',model_initialized=False))
        self.recover_calls=[]
        def recover(**kwargs):
            self.recover_calls.append(True);self.initial['status']='ready'
            for name in ('planner','scene'):self.initial[name]['status']='ready'
        self.service=Playground(self.root/'playground',self.bridge,lambda:deepcopy(self.initial),recover_planner=recover)
        def artifacts(config,output,bridge,speed,*args,**kwargs):
            result={}
            for key,name,value in (('source_path','source.json',self.source),('hardware_binding','hardware-binding.json',self.binding),
                                   ('plan_path','plan.json',{'cpu_fixture':True})):
                path=output/name;path.write_text(json.dumps(value));result[key]=str(path)
            image=output/'frame.jpg';image.write_bytes(b'\xff\xd8CPU fixture\xff\xd9')
            for name in ('planner','scene'):self.initial[name]['model_initialized']=True
            return dict(validated=True,timeline_digest='checked-cpu-plan',first_frame_path=str(image),
                        speed_scale=speed,operator_assistance_digest=self.binding['operator_assistance_digest'],**result)
        self.pipeline=SimpleNamespace(prepare_environment=Mock(return_value=[dict(key='compute',status='ready')]),
            plan_assisted=Mock(side_effect=artifacts),plan=Mock(side_effect=artifacts),
            warmup=Mock(side_effect=AssertionError('no automatic observation warmup')),
            execute=Mock(side_effect=AssertionError('no automatic execution')))

    def owner(self):
        value=owner(deepcopy(self.source['initial_state']['measured_reported']),stamp=time.time_ns())
        value.update(connected=self.connected,holder='collection',recording_configured=True,
            pedal=dict(input_healthy=True,physically_down=False))
        return value

    def health(self):
        return dict(connected=self.connected,owner=self.owner(),
            cameras={name:{'ready':True} for name in ('base_0','left_wrist_0','right_wrist_0')})

    def run_plan(self,payload):
        with patch.dict(sys.modules,playground_pipeline=self.pipeline):
            result=self.service.start(payload)
            self.service.thread.join(3)
            self.assertFalse(self.service.thread.is_alive())
        return result

    def test_cold_explicit_plan_prepares_and_installs_existing_ui_state_without_motion(self):
        with self.assertRaisesRegex(ValueError,'环境准备'):self.service.start({'action':'plan'})
        self.run_plan(dict(action='plan',operator_assisted_request_id=self.request_id))
        state=self.service.snapshot()
        self.assertEqual(state['status'],'ready',state)
        self.assertTrue(state['plan']['validated']);self.assertTrue(state['can_execute'])
        self.assertEqual(state['warmup']['status'],'ready')
        self.assertIn('/api/playground/media/frame',state['plan']['first_frame_url'])
        checkpoint=json.loads((self.service.directory/'prepared-plan.json').read_text())
        self.assertEqual(checkpoint['plan'],state['plan']);self.assertFalse(checkpoint['execution_pending'])
        self.assertEqual(self.recover_calls,[True])
        self.pipeline.plan_assisted.assert_called_once()
        self.assertEqual(self.pipeline.plan_assisted.call_args.args[-1],self.inputs)
        self.pipeline.plan.assert_not_called();self.pipeline.warmup.assert_not_called();self.pipeline.execute.assert_not_called()
        self.assertFalse(self.commands)
        # Registered directory is never an implicit fallback on the next click.
        self.run_plan({'action':'plan'})
        self.pipeline.plan.assert_called_once();self.assertEqual(self.pipeline.plan_assisted.call_count,1)

    def test_wrong_id_environment_or_action_never_starts_compute(self):
        for payload in (dict(action='plan',operator_assisted_request_id='other'),
                        dict(action='execute',operator_assisted_request_id=self.request_id),
                        dict(action='prepare',operator_assisted_request_id=self.request_id)):
            with self.assertRaises(ValueError):self.service.start(payload)
        self.bridge.config['environment']='fake'
        with self.assertRaisesRegex(ValueError,'真机'):
            self.service.start(dict(action='plan',operator_assisted_request_id=self.request_id))
        self.pipeline.prepare_environment.assert_not_called();self.assertFalse(self.commands)
        self.assertIsNone(self.service.thread)

    def test_disconnected_request_never_connects(self):
        self.connected=False
        with self.assertRaisesRegex(ValueError,'fresh healthy'):
            self.service.start(dict(action='plan',operator_assisted_request_id=self.request_id))
        self.pipeline.prepare_environment.assert_not_called();self.assertFalse(self.commands)

    def test_connection_loss_during_service_preparation_does_not_reacquire(self):
        def lose(config,**kwargs):
            self.connected=False
            return [dict(key='compute',status='ready')]
        self.pipeline.prepare_environment.side_effect=lose
        self.run_plan(dict(action='plan',operator_assisted_request_id=self.request_id))
        state=self.service.snapshot();self.assertEqual(state['status'],'failed')
        self.assertFalse(state['can_execute']);self.assertFalse(state['plan']['validated'])
        self.assertEqual(state['warmup']['status'],'failed')
        self.pipeline.plan_assisted.assert_not_called();self.assertFalse(self.commands)

    def test_changed_returned_source_is_not_installed(self):
        build=self.pipeline.plan_assisted.side_effect
        def replace(*args,**kwargs):
            result=build(*args,**kwargs)
            source=deepcopy(self.source);source['run_id']='different-request'
            Path(result['source_path']).write_text(json.dumps(source))
            return result
        self.pipeline.plan_assisted.side_effect=replace
        self.run_plan(dict(action='plan',operator_assisted_request_id=self.request_id))
        self.assertEqual(self.service.snapshot()['status'],'failed')
        self.assertFalse((self.service.directory/'prepared-plan.json').exists())
        self.pipeline.execute.assert_not_called();self.assertFalse(self.commands)


if __name__=='__main__':unittest.main()
