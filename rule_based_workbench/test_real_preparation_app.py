"""CPU fake preparation/HTTP checks; never camera, SDK, GPU or remote calls."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

import app
from integration import _digest
from real_bridge import RealBridge
from real_workflow import load
from test_app import fixture_messages, serving
from test_frozen_executor import fixture


def artifacts(root):
    root.mkdir(parents=True,exist_ok=True)
    source,plan=fixture(.001)
    source={k:deepcopy(v) for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
    serials=dict(head='fixture-h',left='fixture-l',right='fixture-r')
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',robot_id='fixture-robot',camera_serials=serials)
    from joint_space import freeze_reported
    plan['native_actions'] = [deepcopy(tick['target']) for tick in plan['ticks']]
    freeze_reported(plan, source)
    binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture-robot',camera_serials=serials,
                 calibration_profile_digest=_digest(source['calibration_profile']),source_digest=_digest(source))
    for name,value in [('source.json',source),('plan.json',plan),('hardware-binding.json',binding)]:
        (root/name).write_text(json.dumps(value))
    return dict(source_path=str(root/'source.json'),plan_path=str(root/'plan.json'),
                hardware_binding=str(root/'hardware-binding.json'),artifact_dir=str(root))


def setup(root):
    contract=root/'contract.json'; config=root/'pipeline.json'; token=root/'token'
    contract.write_text(json.dumps({'filesystem':{'writable_roots':['/remote/fixture']},'provenance':{'config_sha256':{}}}))
    config.write_text(json.dumps({'remote_run_root':'/remote/fixture','contract_path':str(contract)}))
    token.write_text('test-not-a-real-secret-'*3)
    bridge=RealBridge(dict(robot_id='fixture-robot',hardware_access_enabled=False,
        control_url='http://127.0.0.1:18400',token_file=str(token),
        cameras={camera:dict(url=f'http://127.0.0.1:{18403+n}',serial=serial) for n,(camera,serial) in enumerate(
            zip(('base_0','left_wrist_0','right_wrist_0'),('fixture-h','fixture-l','fixture-r')))},
        preparation=dict(pipeline_config=str(config),capture_dir='/remote/capture',physical_binding='/remote/binding.json')))
    # Any accidental control or camera HTTP access fails the check immediately.
    bridge.request=Mock(side_effect=AssertionError('hardware access forbidden in fake preparation check'))
    bridge.control=Mock(side_effect=AssertionError('preparation must never acquire/confirm/command'))
    bridge.prepare_collection=Mock(side_effect=AssertionError('preparation must not preview/confirm motion'))
    state=app.WorkbenchState(); state.restore(root/'dual')
    source,plan,_=fixture_messages()
    state.pipeline.ingest_source(source); state.pipeline.accept_plan(plan); state.persist()
    return bridge,state


class RealPreparationAppTest(unittest.TestCase):
    def test_trusted_binding_publication(self):
        with TemporaryDirectory() as folder:
            root = Path(folder); bridge, _ = setup(root)
            result = root / 'job/result'; artifacts(result)
            target = root / 'trusted.json'
            new = (result / 'hardware-binding.json').read_bytes()
            old = json.loads(new); old['source_digest'] = 'previous'
            previous = json.dumps(old).encode(); target.write_bytes(previous)
            bridge.config['trusted_binding_path'] = str(target)
            bridge.owner = Mock(return_value=dict(mode='hardware',state='READY', stationary=True, automatic_enabled=False))
            with patch.object(app, 'REAL_BRIDGE', bridge):
                app.publish_real_binding(result)
                self.assertEqual(target.read_bytes(), new)
                self.assertEqual((result / 'hardware-binding.previous.json').read_bytes(), previous)
                receipt = json.loads((result / 'hardware-binding-publication.json').read_text())
                self.assertEqual(receipt['published_sha256'], app.hashlib.sha256(new).hexdigest())
                bridge.control.assert_not_called(); bridge.request.assert_not_called()
                for update in ({'automatic_enabled': True}, {'stationary': False}, {'state': 'RUNNING'},
                               {'motion': {'status': 'VERIFYING'}}, {'acquisition_pending': True}):
                    target.write_bytes(previous)
                    bridge.owner.return_value = dict(mode='hardware',state='READY', stationary=True, automatic_enabled=False) | update
                    with self.assertRaisesRegex(ValueError, 'stationary'):
                        app.publish_real_binding(result)
                    self.assertEqual(target.read_bytes(), previous)
                bridge.owner.return_value = dict(mode='hardware',state='READY', stationary=True, automatic_enabled=False)
                bad = json.loads(new); bad['source_digest'] = 'unvalidated'
                (result / 'hardware-binding.json').write_text(json.dumps(bad))
                with self.assertRaisesRegex(ValueError, 'observation differs'):
                    app.publish_real_binding(result)
                self.assertEqual(target.read_bytes(), previous)

    def test_failed_publication_does_not_promote(self):
        with TemporaryDirectory() as folder:
            root = Path(folder); bridge, _ = setup(root)
            old_config = deepcopy(bridge.config)
            def run(config, output, **kwargs):
                artifacts(output)
                return {'status': 'prepared'}
            with patch.multiple(app, REAL_BRIDGE=bridge, REAL_JOB={'status': 'running'}, REAL_STATE_DIR=root / 'state'), \
                 patch('real_pipeline.run', side_effect=run), patch.object(app, 'persist_real_job'), \
                 patch.object(app, 'publish_real_binding', side_effect=ValueError('publication refused')):
                app.run_real_preparation(Path('pipeline'), root / 'job', bridge.config['preparation'])
                self.assertEqual(app.REAL_JOB['status'], 'failed')
                self.assertEqual(bridge.config, old_config)
                bridge.control.assert_not_called(); bridge.request.assert_not_called()

    def test_automatic_observation_gate_and_dispatch(self):
        with TemporaryDirectory() as folder:
            root = Path(folder); bridge, state = setup(root)
            pipeline_config = bridge.config['preparation']['pipeline_config']
            bridge.config['preparation'] = dict(pipeline_config=pipeline_config, acquisition={})
            bridge.camera = Mock(return_value=(b'atomic-zip', 'application/zip'))
            def acquire(config, output, *, read_snapshot, read_owner):
                self.assertEqual(read_snapshot(), b'atomic-zip')
                return dict(pipeline_config=str(output/'sealed.json'), capture_dir='/sealed/new', physical_binding='/sealed/new/binding.json')
            def run(config, output, **kwargs):
                self.assertEqual(kwargs['capture_dir'], '/sealed/new')
                artifacts(output)
                return {'status': 'prepared'}
            with patch.multiple(app, REAL_BRIDGE=bridge, REAL_JOB=None, REAL_STATE_DIR=root/'real'), \
                    patch('real_acquisition.validate_config'), patch('real_acquisition.prepare_inputs', side_effect=acquire) as capture, \
                    patch('real_pipeline.run', side_effect=run) as pipeline, patch.object(app, 'Thread') as worker:
                with self.assertRaisesRegex(ValueError, '相机访问窗口'):
                    app.start_real_preparation()
                bridge.camera.assert_not_called()
                bridge.config['hardware_access_enabled'] = True
                app.start_real_preparation()
                pipeline.assert_not_called()
                worker.call_args.kwargs['target'](*worker.call_args.kwargs['args'])
                self.assertEqual(app.REAL_JOB['status'], 'prepared')
                capture.assert_called_once()
                bridge.camera.assert_called_once_with('base_0', 'snapshot.zip')
                bridge.control.assert_not_called()

    def test_http_dispatch_success_and_busy_mode_boundary(self):
        with TemporaryDirectory() as folder:
            root=Path(folder); bridge,state=setup(root)
            original_dsw=(state.directory/'state.json').read_bytes()
            def fake_run(config,output,**kwargs):
                if kwargs.get('dry_run'): return {'status':'dry-run'}
                artifacts(output)
                return {'status':'prepared'}
            with patch.multiple(app,REAL_BRIDGE=bridge,REAL_JOB=None,REAL_STATE_DIR=root/'dual/real-mode',
                                WORKBENCH_MODE='real',PIPELINE_CONFIG=None,DETECTOR_MANAGER=None), \
                    patch('real_pipeline.run',side_effect=fake_run) as pipeline, patch.object(app,'Thread') as worker, \
                    serving(state) as request:
                csrf={'X-Workbench-CSRF':app.CSRF_TOKEN}
                request('POST','/api/pipeline/run',{'mode':'real'},expected=403)
                self.assertFalse(worker.called)
                started=request('POST','/api/pipeline/run',{'mode':'real'},headers=csrf,expected=202)
                self.assertEqual(started['status'],'running')
                self.assertEqual(pipeline.call_count,1)
                self.assertTrue(pipeline.call_args.kwargs['dry_run'])
                self.assertTrue(request('GET','/api/mode')['busy'])
                request('POST','/api/mode',{'mode':'dual_dsw'},headers=csrf,expected=400)
                request('POST','/api/pipeline/run',{'mode':'real'},headers=csrf,expected=400)
                self.assertEqual(worker.call_count,1)
                call=worker.call_args
                call.kwargs['target'](*call.kwargs['args'])
                self.assertEqual(app.REAL_JOB['status'],'prepared')
                summary=request('GET','/api/state?mode=real')
                self.assertEqual(summary['job']['status'],'prepared')
                self.assertEqual(summary['trajectory']['state'],'validated')
                self.assertEqual(pipeline.call_count,2)
                saved=json.loads((app.REAL_STATE_DIR/'state.json').read_text())
                self.assertEqual(saved['job']['status'],'prepared')
                self.assertEqual(saved['artifacts']['source_path'],bridge.config['source_path'])
                self.assertEqual((state.directory/'state.json').read_bytes(),original_dsw)
                bridge.request.assert_not_called(); bridge.control.assert_not_called(); bridge.prepare_collection.assert_not_called()

    def test_failed_run_clears_old_real_and_never_falls_back_to_dsw(self):
        with TemporaryDirectory() as folder:
            root=Path(folder); bridge,state=setup(root)
            bridge.config.update(artifacts(root/'old-success'))
            self.assertIsNotNone(load(bridge.config)['plan'])
            original_dsw=(state.directory/'state.json').read_bytes()
            def fake_run(config,output,**kwargs):
                return {'status':'dry-run'} if kwargs.get('dry_run') else {'status':'failed','error':'fake detector failure'}
            with patch.multiple(app,REAL_BRIDGE=bridge,REAL_JOB=None,REAL_STATE_DIR=root/'dual/real-mode',
                                WORKBENCH_MODE='real',PIPELINE_CONFIG=None,DETECTOR_MANAGER=None), \
                    patch('real_pipeline.run',side_effect=fake_run), patch.object(app,'Thread') as worker, serving(state) as request:
                app.start_real_preparation()
                self.assertIsNone(load(bridge.config)['source'])
                worker.call_args.kwargs['target'](*worker.call_args.kwargs['args'])
                summary=request('GET','/api/state?mode=real')
                self.assertEqual(summary['job']['status'],'failed')
                self.assertIn('fake detector failure',summary['job']['error'])
                self.assertIsNone(summary['scene']); self.assertIsNone(summary['trajectory'])
                self.assertIsNone(summary['pipeline']['source_run_id'])
                self.assertEqual((state.directory/'state.json').read_bytes(),original_dsw)
                saved=json.loads((app.REAL_STATE_DIR/'state.json').read_text())
                self.assertIsNone(saved['artifacts']['source_path'])
                self.assertIsNone(saved['artifacts']['plan_path'])
                bridge.request.assert_not_called(); bridge.control.assert_not_called(); bridge.prepare_collection.assert_not_called()

    def test_restart_restores_cleared_artifact_paths_and_requires_review(self):
        with TemporaryDirectory() as folder:
            root=Path(folder); bridge,state=setup(root)
            old=artifacts(root/'old-success'); bridge.config.update(old)
            real_state=state.directory/'real-mode'; real_state.mkdir()
            cleared={key:None for key in ('source_path','plan_path','hardware_binding','artifact_dir')}
            (real_state/'state.json').write_text(json.dumps({'job':{'status':'running','directory':str(root/'interrupted')},'artifacts':cleared}))
            real_config=root/'real-config.json'; real_config.write_text(json.dumps(bridge.config))
            with patch.multiple(app,STATE=state,REAL_BRIDGE=None,REAL_JOB=None,REAL_STATE_DIR=None,
                                WORKBENCH_MODE='real',PIPELINE_CONFIG=None,DETECTOR_MANAGER=None), \
                    patch('sys.argv',['app','--state-dir',str(state.directory),'--real-config',str(real_config)]), \
                    patch('real_bridge.RealBridge',return_value=bridge),patch('signal.signal'), \
                    patch.object(app,'ThreadingHTTPServer') as server:
                server.return_value.serve_forever.side_effect=KeyboardInterrupt
                app.main()
                self.assertEqual(app.REAL_JOB['status'],'needs-review')
                self.assertIsNone(load(bridge.config)['source'],'restart must not resurrect cleared paths from real-config')
                self.assertIsNone(load(bridge.config)['plan'])
                bridge.request.assert_not_called(); bridge.control.assert_not_called()


    def test_confirmed_execution_finalizes_only_new_recording(self):
        bridge=Mock()
        bridge.config={'raw_source_dir':'/fixture/raw'}
        plan={'duration_s':1,'timeline_digest':'current'}
        old={'connected':True,'frozen':{'timeline_digest':'current'},'recording':{'done':True,'directory':'old'}}
        new={**old,'recording':{'done':True,'directory':'new'}}
        bridge.owner.side_effect=[old,new]
        with patch.multiple(app,REAL_BRIDGE=bridge,REAL_JOB={'status':'running'},REAL_STATE_DIR=None), \
                patch('real_workflow.finalize_collection',return_value={'status':'ready'}) as finalize:
            app.monitor_real_execution(plan,'old',sleep=lambda _:None)
            self.assertEqual(app.REAL_JOB['status'],'completed')
            finalize.assert_called_once_with(bridge.config,new['recording'])
            bridge.control.assert_not_called()
        bridge.config={};bridge.owner.side_effect=None;bridge.owner.return_value=new
        with patch.multiple(app,REAL_BRIDGE=bridge,REAL_JOB={'status':'running'},REAL_STATE_DIR=None), \
                patch('real_workflow.finalize_collection') as finalize:
            app.monitor_real_execution(plan,'old',sleep=lambda _:None)
            self.assertEqual(app.REAL_JOB['status'],'awaiting-artifacts')
            finalize.assert_not_called()
        bridge.owner.return_value={**new,'frozen':{'timeline_digest':'different-run'}}
        with patch.multiple(app,REAL_BRIDGE=bridge,REAL_JOB={'status':'running'},REAL_STATE_DIR=None):
            app.monitor_real_execution(plan,'old',sleep=lambda _:None)
            self.assertEqual(app.REAL_JOB['status'],'needs-review')


if __name__=='__main__': unittest.main()
