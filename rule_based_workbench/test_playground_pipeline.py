"""Fake inputs and transports only; checks orchestration without GPU, SSH or motors."""
from contextlib import ExitStack
from copy import deepcopy
import json
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

import playground_pipeline as pipeline


def main():
    real_popen=subprocess.Popen
    from run_pipeline import isaac_environment
    gpu_cfg=dict(remote_run_root='/run',workbench_code='/wb',native_code='/native',assets='/assets',deps='/deps',
                 planning_backend='motiongen',curobo_root='/curobo',finger_envelope='/finger',finger_envelope_sha256='a'*64)
    environment=isaac_environment(gpu_cfg,'dsw-scene')
    assert environment['PYTHONPATH'].startswith('/curobo/python:/curobo/deps:')
    with patch.object(pipeline,'gpu_check_command',return_value='gpu-check'):
        command=pipeline.native_stage(gpu_cfg,'replay','/output',plan_path='/plan')['argv'][-1]
        preset=dict(preset_id='soft',stiffness_n_m=329.24414834950727,damping_n_s_m=10.,diagnostic_claw_compression=True)
        for mode in ('geometry-inputs','replay'):
            fake=pipeline.native_stage(dict(gpu_cfg,environment='fake',cup_contact_preset=preset),mode,'/output')['argv'][-1]
            assert '--cup-contact-preset' in fake and str(preset['stiffness_n_m']) in fake
        physical=pipeline.native_stage(dict(gpu_cfg,environment='physical',cup_contact_preset=preset),'replay','/output')['argv'][-1]
        assert '--cup-contact-preset' not in physical
    assert all(key+'='+value in command for key,value in environment.items())
    assert pipeline.controller_failure({'reason':'host stale','motion':{'reason':'operator stop'}},{'controller_outcome':'failed'})=='host stale'
    assert pipeline.controller_failure({'frozen':{'status':'failed','reason':'tracking failure'}},{})=='tracking failure'
    assert pipeline.controller_failure({'last_fault_diagnostic':{'error':'old fault'}},{'controller_outcome':'failed'}) is None
    # Both the active row and the last completed/failed receipt must reach the UI.
    with TemporaryDirectory() as folder, patch.object(pipeline, 'bounded_command', return_value=(['fake'], 1)), \
            patch.object(pipeline.subprocess, 'run', return_value=SimpleNamespace(returncode=0)) as command, \
            patch.object(pipeline,'fetch',side_effect=lambda remote,local,**kw:Path(local).write_text(json.dumps({'status':'geometric-plan-ready'}))):
        events = []
        emit = lambda label, **fields: events.append(dict(label=label, **fields))
        stages = [dict(name='detect'), dict(name='playground-plan',result_path='/native/result.json')]
        pipeline.run_stages(stages, folder, {'resources': {'max_wall_time_minutes': 1}}, emit)
        assert events[0]['stages'][0]['status'] == 'running'
        assert events[-1]['stages'][-1]['status'] == 'completed'
        assert events[-1]['stages'][-1]['label'] == 'CPU 轨迹规划与碰撞检查'
        assert len(events[-1]['stages']) == 2
        assert events[0]['next_label'] == 'CPU 轨迹规划与碰撞检查'
        command.return_value.returncode = 1
        try:
            pipeline.run_stages(stages[:1], folder, {'resources': {'max_wall_time_minutes': 1}}, emit)
            raise AssertionError('failed stage accepted')
        except RuntimeError:
            pass
        assert events[-1]['stage_status'] == 'failed' and events[-1]['stages'][-1]['status'] == 'failed'
    # A silent native exit 0 is failure until its own completion receipt is present.
    for payload in (None, 'broken json', '[]', '{"status":"failed","error":"renderer stopped"}', '{"status":"first-frame-ready"}'):
        with TemporaryDirectory() as folder, patch.object(pipeline,'bounded_command',return_value=(['fake'],1)), \
                patch.object(pipeline.subprocess,'run',return_value=SimpleNamespace(returncode=0)) as command:
            def fetched(remote,local,**kw):
                if payload is None:raise FileNotFoundError(remote)
                Path(local).write_text(payload)
            events=[]
            with patch.object(pipeline,'fetch',side_effect=fetched):
                try:pipeline.run_stages([dict(name='playground-geometry-inputs',result_path='/native/result.json'),dict(name='next')],folder,{'resources':{'max_wall_time_minutes':1}},lambda label,**kw:events.append(kw))
                except RuntimeError:pass
                else:raise AssertionError('missing/failed/wrong native receipt accepted')
            receipts=json.loads((Path(folder)/'timings.json').read_text())
            assert len(receipts)==1 and receipts[0]['status']=='failed'
            assert command.call_count==1, 'downstream stage ran despite missing native result'
            assert events[-1]['stage_status']=='failed'
    for name,status in pipeline.NATIVE_STAGE_STATUS.items():
        with TemporaryDirectory() as folder, patch.object(pipeline,'bounded_command',return_value=(['fake'],1)), \
                patch.object(pipeline.subprocess,'run',return_value=SimpleNamespace(returncode=0)), \
                patch.object(pipeline,'fetch',side_effect=lambda remote,local,**kw:Path(local).write_text(json.dumps({'status':status}))):
            rows=pipeline.run_stages([dict(name=name,result_path='/native/result.json')],folder,{'resources':{'max_wall_time_minutes':1}})
            assert rows[0]['status']=='completed' and rows[0]['native_status']==status
    source = dict(run_id='fresh-source', scene={'id': 'real-five-second-window'})
    plan = dict(run_id='fresh-plan', timeline_digest='checked-timeline', ticks=[{}], duration_s=.1,
                execution_derivation={'speed_scale': .3})
    binding = dict(robot_id='robot', camera_serials={'head': 'camera'})
    service = dict(root='/detector-template', socket='/detector-template/socket', owner_token='old-owner', identity='detector-v1')
    cfg = dict(remote_run_root='/remote/run', workbench_code='/candidate', native_code='/native', assets='/assets', detector_service=service)
    contract = dict(filesystem={'writable_roots': ['/remote/run', '/detector-template']}, resources={'max_wall_time_minutes': 10},
                    detector_service=service, provenance={'code_sha256': {'/candidate/playground_sim.py': 'frozen-pin'}})
    stage_names = ['preflight-dsw-scene','initialize','shared-storage','physical-capture-import','detect',
                   'physical-detection-bind','bridge','physical-model-bind','geometry','geometry-verify','place',
                   'export','replay-source','replay-target','physical-target-replay','export-target-playback','fetch-source.json']
    with TemporaryDirectory() as folder, ExitStack() as stack:
        root = Path(folder)
        configured = dict(pipeline_config=str(root/'candidate.json'))
        bridge = SimpleNamespace(config={'preparation': {'pipeline_config': str(root/'active.json')}, 'local_recording_root': str(root/'recordings'),
                                         'raw_source_dir': '/obsolete/raw'})
        def load(path):
            if path.is_file():
                data = json.loads(path.read_text())
                return data, json.loads(Path(data['contract_path']).read_text()), {}
            data, bound = deepcopy(cfg), deepcopy(contract)
            if path.name == 'active.json':
                data['detector_service'].update(root='/active-detector', socket='/active-detector/socket', owner_token='active-owner')
                bound['detector_service'] = deepcopy(data['detector_service'])
            return data, bound, {}

        stack.enter_context(patch.object(pipeline, 'load_config', side_effect=load))
        acquire = stack.enter_context(patch('real_acquisition.prepare_inputs', side_effect=lambda acquisition,*args,**kw:{
            'pipeline_config': acquisition['pipeline_config'], 'capture_dir': '/fresh/capture', 'physical_binding': '/fresh/binding'}))
        bridge.camera = Mock(return_value=(b'fake-rgbd', 'application/zip'))
        bridge.owner = Mock(return_value={})
        preparation = stack.enter_context(patch('real_pipeline.preparation_stages', return_value=[dict(name=n) for n in stage_names[:8]]))
        native = stack.enter_context(patch.object(pipeline, 'native_stage', side_effect=lambda c,m,o,**kw:
                                                 dict(name='playground-'+m, argv=['fake-'+m], timeout_s=1)))
        geometry = stack.enter_context(patch.object(pipeline, 'geometry_stage', side_effect=lambda c,o,**kw:
                                                   dict(name='playground-plan', argv=['fake-cpu-plan'], timeout_s=1)))
        run = stack.enter_context(patch.object(pipeline, 'run_stages'))
        validate = stack.enter_context(patch('real_contract.validate_real_plan'))
        evidence = stack.enter_context(patch('frozen_executor.validate_plan_evidence'))

        def fetch_plan(remote, local, **kw):
            value = (dict(status='geometric-plan-ready') if remote.endswith('/result.json') else source
                     if remote.endswith('/source.json') else binding if remote.endswith('/hardware-binding.json') else plan)
            Path(local).write_text(json.dumps(value))

        transport = stack.enter_context(patch.object(pipeline, 'fetch', side_effect=fetch_plan))
        prepared = pipeline.plan(configured, root/'planning', bridge, .3)
        assert acquire.call_count == 1
        assert preparation.call_args.kwargs == {'scene_only': True}
        selected = [s['name'] for s in run.call_args.args[0]]
        assert selected == stage_names[:8] + ['playground-geometry-inputs','playground-plan'], selected
        assert not any(n in selected for n in stage_names[8:]), 'planning must never run old rollout/replay'
        assert prepared['validated'] and validate.call_count == 1 and evidence.call_count == 1
        assert native.call_args.args[1] == 'geometry-inputs'
        assert native.call_count == 1 and geometry.call_count == 1
        assert geometry.call_args.kwargs['speed'] == .3 and 'initial' not in geometry.call_args.kwargs
        actual_cfg = json.loads((root/'planning/config.json').read_text())
        actual_contract = json.loads((root/'planning/contract.json').read_text())
        assert actual_cfg['detector_service']['root'] == '/active-detector'
        assert actual_cfg['detector_service'] == actual_contract['detector_service']
        assert '/active-detector' in actual_contract['filesystem']['writable_roots']
        assert '/detector-template' not in actual_contract['filesystem']['writable_roots']
        assert actual_contract['provenance']['code_sha256'] == contract['provenance']['code_sha256']
        # An exit code of zero is insufficient: reject a failed native receipt before fetching/promoting a plan.
        validation_count, evidence_count = validate.call_count, evidence.call_count
        fetched = []

        def failed_receipt(remote, local, **kw):
            fetched.append(remote)
            assert remote.endswith('/result.json'), 'failed receipt must stop all downstream fetches'
            Path(local).write_text(json.dumps(dict(status='failed', error='native planner failed before shutdown')))

        transport.side_effect = failed_receipt
        try:
            pipeline.plan(configured, root/'failed-planning', bridge, .3)
            raise AssertionError('failed native receipt accepted')
        except ValueError as error:
            assert 'native planner failed' in str(error)
        assert len(fetched) == 1 and validate.call_count == validation_count and evidence.call_count == evidence_count
        transport.side_effect = fetch_plan

        actions = []
        lanes = []
        raw = root/'recordings/current/raw'
        raw.mkdir(parents=True)
        (raw/'binding.json').write_text(json.dumps(binding))

        recording_fields={};owner_fields={};preview_fields={};owner_sequence=[]
        class Lane:
            def __init__(self, config):
                self.config, self.confirmed = config, False
                lanes.append(self)
                self.instance=f'{len(lanes):032x}';self.acceptance_read=False

            def owner(self):
                status=dict(owner_fields)
                if self.config.get('environment')=='fake':
                    instance=self.instance if self.confirmed else '0'*32
                    status.update(execution_instance_id=instance,runtime_nonce='owner-runtime',command_epoch=3,
                        execution_calibration=dict(execution_instance_id=instance,command_epoch=3,plan_sha256=['a'*64]))
                acceptance=self.config.get('environment')=='fake' and self.confirmed and not self.acceptance_read
                if acceptance:self.acceptance_read=True
                elif self.confirmed and owner_sequence:status.update(owner_sequence.pop(0))
                return dict(connected=True, **status,recording=dict(directory=str(raw) if self.confirmed else '/previous/raw', done=self.confirmed)|recording_fields)

            def control(self, command):
                actions.append(command['action'])
                if command['action'] == 'confirm':
                    self.confirmed = True
                    return {'queued':'confirm','plan_sha256':'a'*64,'sequence_id':'a'*64}
                return dict(token='single-use-preview', timeline_digest=plan['timeline_digest'],**preview_fields)

        stack.enter_context(patch('real_bridge.RealBridge', Lane))
        publication = stack.enter_context(patch.object(pipeline, 'publish_binding'))
        raw_resolver = stack.enter_context(patch('real_workflow.resolve_raw_source', return_value=raw))
        raw_verify = stack.enter_context(patch('real_recording.verify_raw', return_value=(
            {'controller_outcome': 'completed','controller_reason':'completed', 'validation': {'status': 'passed'},'camera_frames':dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),2)}, source, plan)))

        def export(raw, target, output, **kwargs):
            assert callable(kwargs['runner'])
            output.mkdir()
            for camera in ('base_0', 'left_wrist_0', 'right_wrist_0'):
                (output/f'source-{camera}.mp4').write_bytes(b'fake-video')

        exporter = stack.enter_context(patch('real_recording.export_playback', side_effect=export))
        process = Mock()
        process.wait.return_value = 0
        process.poll.return_value = 0
        popen = stack.enter_context(patch.object(pipeline.subprocess, 'Popen', return_value=process))
        upload = stack.enter_context(patch.object(pipeline.subprocess, 'run', return_value=SimpleNamespace(returncode=0)))

        # Speed changes must check their native receipt before binding publication or any preview/motion.
        transport.side_effect = failed_receipt
        changed_speed = dict(prepared, speed_scale=.2)
        validation_count, evidence_count = validate.call_count, evidence.call_count
        native_before, geometry_before, acquisition_before = native.call_count, geometry.call_count, acquire.call_count
        try:
            pipeline.execute(configured, root/'failed-speed-change', bridge, changed_speed)
            raise AssertionError('failed speed-change receipt accepted')
        except ValueError as error:
            assert 'native planner failed' in str(error)
        assert actions == [] and publication.call_count == 0 and popen.call_count == 0
        assert validate.call_count == validation_count and evidence.call_count == evidence_count
        assert native.call_count == native_before and geometry.call_count == geometry_before+1
        assert acquire.call_count == acquisition_before, 'speed change must not observe again'
        assert geometry.call_args.kwargs['speed'] == .2
        assert geometry.call_args.kwargs['initial'].endswith('/playground-plan/initial-plan.json')

        def fetch_replay(remote, local, **kw):
            if remote.endswith('/result.json'):
                Path(local).write_text(json.dumps(dict(status='completed', source_run_id=source['run_id'],
                    plan_run_id=plan['run_id'], timeline_digest=plan['timeline_digest'])))
            else:
                Path(local).write_bytes(b'fake-sim-video')

        transport.side_effect = fetch_replay
        # Failed pre-action remote preparation cannot reach preview/confirm.
        plan_file=Path(prepared['plan_path']);plan_file.write_text(json.dumps(dict(plan,execution_resume={'test':True})))
        with patch.object(pipeline,'prepare_remote_resume',side_effect=TimeoutError('transport')):
            try:pipeline.execute(configured,root/'failed-remote-resume',bridge,prepared)
            except TimeoutError:pass
            else:raise AssertionError('transport error was hidden')
        assert not actions and publication.call_count==0
        assert json.loads((root/'failed-remote-resume/execution-phase.json').read_text())==dict(phase='pre_action',timeline_digest=plan['timeline_digest'])
        plan_file.write_text(json.dumps(plan))
        owner_fields.update(reason='completed',motion={'reason':'completed'},frozen={'status':'completed','reason':'completed'},last_fault_diagnostic={'error':'historical fault'})
        result = pipeline.execute(configured, root/'execution', bridge, prepared)
        assert not result['errors'], 'informational or historical reason made a success fail'
        owner_fields.clear()
        assert actions == ['preview_frozen', 'confirm'], actions
        assert popen.call_count == 1 and publication.call_count == 1
        assert upload.call_count==0, 'unchanged DSW parent must not be uploaded again'
        assert native.call_args.kwargs['plan_path'].endswith('/playground-plan/plan.json')
        assert result['status'] == 'completed' and len(result['videos']) == 6
        assert 'raw_source_dir' not in raw_resolver.call_args.args[0], 'do not export stale raw'
        assert raw_resolver.call_args.args[1]['directory'] == str(raw)
        assert exporter.call_args.args[0] == raw

        # Successful speed changes rebuild/audit the cached route with CPU geometry only;
        # the only Kit invocation is the separately requested action replay.
        actions.clear()
        native_before, geometry_before, acquisition_before = native.call_count, geometry.call_count, acquire.call_count
        previous_plan = deepcopy(plan)
        plan.update(timeline_digest='checked-timeline-20pct', execution_derivation={'speed_scale': .2})

        def fetch_changed(remote, local, **kw):
            if '/playground-speed-' in remote:
                return fetch_plan(remote, local, **kw)
            return fetch_replay(remote, local, **kw)

        transport.side_effect = fetch_changed
        result = pipeline.execute(configured, root/'changed-speed', bridge, changed_speed)
        assert result['status'] == 'completed' and actions == ['preview_frozen','confirm']
        assert native.call_count == native_before+1 and native.call_args.args[1] == 'replay'
        assert geometry.call_count == geometry_before+1 and acquire.call_count == acquisition_before
        assert '/playground-speed-' in native.call_args.kwargs['plan_path']
        assert geometry.call_args.kwargs['initial'].endswith('/playground-plan/initial-plan.json')
        plan.clear(); plan.update(previous_plan)
        transport.side_effect = fetch_replay

        # Any upfront evidence failure occurs before publishing/previewing/launching either lane.
        actions.clear(); popen.reset_mock(); publication.reset_mock()
        validate.side_effect = ValueError('invalid checked plan')
        try:
            pipeline.execute(configured, root/'invalid', bridge, prepared)
            raise AssertionError('invalid plan accepted')
        except ValueError as error:
            assert 'invalid checked plan' in str(error)
        assert actions == [] and popen.call_count == 0 and publication.call_count == 0
        validate.side_effect = None

        # Simulation failure must not lose the already exported three physical recordings.
        actions.clear()
        transport.side_effect = OSError('simulation result missing')
        result = pipeline.execute(configured, root/'simulation-failed', bridge, prepared)
        assert result['status'] == 'failed' and len(result['videos']) == 3
        assert all(Path(v['path']).is_file() for v in result['videos'])
        assert actions == ['preview_frozen', 'confirm'], 'failure never retries motion'

        # Startup failed before any frames: preserve the original control/recording
        # errors, skip raw export, and never replace them with missing-video errors.
        owner_fields.update(reason='host feedback is older than 0.1 s',motion={'reason':'operator stop'},
                            last_fault_diagnostic={'error':'historical unrelated fault'})
        recording_fields.update(ready=False,error='three native raw streams did not start within 5 seconds')
        failed_manifest=dict(controller_outcome='failed',controller_reason='host feedback is older than 0.1 s',
            camera_frames=dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),0))
        recording_fields['manifest']=failed_manifest
        raw_verify.return_value=(failed_manifest,source,plan)
        exported_before=exporter.call_count;actions.clear()
        failure_progress=[];wait_checked=[]
        def failed_progress(label, **fields):
            failure_progress.append(deepcopy(fields))
        def wait_after_feedback(**kwargs):
            assert failure_progress[-1]['physical_outcome']=='failed'
            assert failure_progress[-1]['errors']['controller']=='host feedback is older than 0.1 s'
            assert failure_progress[-1]['recording']['directory']==str(raw)
            wait_checked.append(True)
            return 0
        process.wait.side_effect=wait_after_feedback
        result=pipeline.execute(configured,root/'zero-frame-startup',bridge,prepared,progress=failed_progress)
        process.wait.side_effect=None
        assert wait_checked == [True], 'current failure must be published before waiting for simulation'
        assert exporter.call_count==exported_before
        assert result['status']=='failed' and result['videos']==[]
        assert result['errors']['controller']=='host feedback is older than 0.1 s'
        assert result['errors']['recording']==recording_fields['error']
        assert result['error'].startswith('controller: host feedback')
        assert actions==['preview_frozen','confirm']
        # Nonzero incomplete recordings remain useful even when the task failed.
        failed_manifest['camera_frames']=dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),2)
        result=pipeline.execute(configured,root/'failed-with-frames',bridge,prepared)
        assert len(result['videos'])==3 and result['errors']['controller']=='host feedback is older than 0.1 s'
        assert exporter.call_count==exported_before+1
        # A batch exporter can fail after producing one valid stream. Keep only
        # independently playable files and attach the export error as secondary.
        def partial_export(raw,target,output,**kwargs):
            output.mkdir();(output/'source-base_0.mp4').write_bytes(b'fake-video')
            raise ValueError('wrist encoder failed')
        exporter.side_effect=partial_export
        with patch.object(pipeline.subprocess,'run',return_value=SimpleNamespace(stdout=json.dumps({'streams':[{'codec_name':'h264','pix_fmt':'yuv420p','nb_read_frames':'2'}]}))):
            result=pipeline.execute(configured,root/'partial-physical-export',bridge,prepared)
        assert len(result['videos'])==1 and result['errors']['physical_export']=='wrist encoder failed'
        assert result['error'].startswith('controller: host feedback')
        exporter.side_effect=export;recording_fields.clear();owner_fields.clear()
        # A sealed recording from another source is not exported or promoted.
        raw_verify.return_value = ({'controller_outcome': 'completed'}, {'run_id': 'stale-source'}, plan)
        exported_before = exporter.call_count
        try:
            pipeline.execute(configured, root/'wrong-raw', bridge, prepared)
            raise AssertionError('stale source accepted')
        except ValueError as error:
            assert 'another run' in str(error)
        assert exporter.call_count == exported_before
        # Capture can fail early while the controller completes. Both lanes must
        # consume the private 30Hz initialization+parent sequence, and no new stop.
        preview_fields['compiled_tick_count_before_initialization']=1
        owner_fields.clear();recording_fields.clear();actions.clear()
        owner_sequence.extend([{'frozen':{'status':'running'}},{'frozen':{'status':'completed'}}])
        recording_fields.update(error='camera stalled',manifest={'controller_outcome':'unverified'})
        capture=dict(controller_outcome='unverified',controller_reason=None,validation={'status':'failed'},
                     camera_frames=dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),2))
        raw_verify.return_value=(capture,source,plan)
        runtime={'timeline_digest':'shared-execution','duration_s':65.,'initialization_duration_s':64.9}
        saved_derivation=b'{"fixture":"verified by dedicated L3 tests"}'
        stack.enter_context(patch.object(pipeline,'verify_l3_derivation',return_value={'passed':True}))
        def fetch_shared(remote,local,**kw):
            if remote.endswith('/l3-initialization-derivation.json'):
                Path(local).write_bytes(saved_derivation);return
            fetch_replay(remote,local,**kw)
            if remote.endswith('/result.json'):
                value=json.loads(Path(local).read_text());value['execution_timeline_digest']='shared-execution'
                value.update(l3_derivation_ref='l3-initialization-derivation.json',
                    l3_derivation_sha256=__import__('hashlib').sha256(saved_derivation).hexdigest())
                Path(local).write_text(json.dumps(value))
        transport.side_effect=fetch_shared
        with patch.object(pipeline,'execution_runtime',return_value=runtime), \
             patch.object(pipeline,'prepare_remote_runtime',return_value='/remote/shared-runtime.json') as derivative, \
             patch.object(pipeline.time,'sleep'):
            result=pipeline.execute(configured,root/'capture-failed-controller-completed',bridge,prepared)
        assert not owner_sequence, 'sealing the failed capture ended monitoring before control completion'
        assert actions==['preview_frozen','confirm'] and derivative.call_count==1
        assert result['physical_outcome']=='completed' and result['capture_outcome']=='failed'
        assert result['status']=='failed' and 'controller' not in result['errors'] and len(result['videos'])==6
        assert native.call_args.kwargs['execution_timeline_path']=='/remote/shared-runtime.json'
        # Local fake non-realtime execution uses one frozen wall deadline, not
        # simulation duration + 60 s. L3 replay cannot qualify same-plant tasks.
        fake_cfg=json.loads(Path(prepared['config_path']).read_text())
        fake_contract=json.loads(Path(fake_cfg['contract_path']).read_text())
        fake_cfg.update(environment='fake',execution={'transport':'local'},resident_deadline_wall_ns=900_000_000_000,
                        contract_path=str(root/'fake-contract.json'))
        fake_contract['resources']['resident_deadline_wall_ns']=fake_cfg['resident_deadline_wall_ns']
        (root/'fake-config.json').write_text(json.dumps(fake_cfg));(root/'fake-contract.json').write_text(json.dumps(fake_contract))
        fake_prepared=dict(prepared,environment='fake',config_path=str(root/'fake-config.json'))
        bridge.config['environment']='fake'
        owner_fields.clear();recording_fields.clear();actions.clear()
        owner_fields.update(feedback_healthy=True,execution_timeline={'command_count':20},
                            native_task={'task_outcome':'success','task_qualified':True,'receipt_path':'unverified-owner-pointer'})
        owner_sequence.extend([dict(frozen={'status':'running','send_count':i},automatic_enabled=True) for i in range(16)])
        owner_sequence.append(dict(frozen={'status':'completed','send_count':20},automatic_enabled=False))
        raw_verify.return_value=(dict(controller_outcome='completed',validation={'status':'passed'},
            camera_frames=dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),2)),source,plan)
        clock=[0.];events=[]
        with patch.object(pipeline,'execution_runtime',return_value=runtime), \
             patch.object(pipeline,'prepare_remote_runtime',return_value='/remote/shared-runtime.json'), \
             patch.object(pipeline.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(pipeline.time,'time_ns',return_value=0), \
             patch.object(pipeline.time,'sleep',side_effect=lambda _:clock.__setitem__(0,clock[0]+10)):
            result=pipeline.execute(configured,root/'resident-slow',bridge,fake_prepared,
                                    progress=lambda label,**kw:events.append(dict(label=label,**kw)))
        assert clock[0]==160 and not owner_sequence and result['status']=='completed'
        assert result['task_outcome']=='unverified', 'unvalidated Owner pointer or independent L3 receipt qualified L2 task'
        assert result['simulation']['scope']=='L3_auxiliary_independent_target' and result['simulation']['task_success_evidence'] is False
        assert result['native_task']['receipt_verified'] is False
        waits=[row['execution_wait'] for row in events if row.get('execution_wait')]
        assert waits[-1]['budget_s']==600 and waits[-1]['commands_sent']==20 and waits[-1]['percent']==100
        assert all(row.get('estimate_s') is None for row in events if 'estimate_s' in row)
        assert all(video['label'].startswith('L3 辅助') for video in result['videos'][3:])
        # Expiring resident wins over the job budget; absent lifetime is never an infinite wait.
        with patch.object(pipeline.time,'time_ns',return_value=800_000_000_000):
            assert pipeline.resident_wait_budget(fake_cfg,fake_contract)==55
        with patch.object(pipeline.time,'time_ns',return_value=900_000_000_000):
            try:pipeline.resident_wait_budget(fake_cfg,fake_contract)
            except TimeoutError:pass
            else:raise AssertionError('expired resident accepted')
        owner_fields.update(automatic_enabled=True,frozen={'status':'running','send_count':0})
        clock[0]=0.;actions.clear()
        with patch.object(pipeline,'execution_runtime',return_value=runtime), \
             patch.object(pipeline,'prepare_remote_runtime',return_value='/remote/shared-runtime.json'), \
             patch.object(pipeline.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(pipeline.time,'time_ns',return_value=0), \
             patch.object(pipeline.time,'sleep',side_effect=lambda _:clock.__setitem__(0,clock[0]+200)):
            try:pipeline.execute(configured,root/'resident-timeout',bridge,fake_prepared)
            except TimeoutError as error:assert 'frozen wall budget' in str(error)
            else:raise AssertionError('non-advancing resident waited without a bound')
        assert clock[0]==600 and actions==['preview_frozen','confirm','stop']
        # Cancellation during compilation must also stop the local active Owner.
        actions.clear()
        with patch.object(pipeline,'execution_runtime',side_effect=InterruptedError('cancel compilation')), \
             patch.object(pipeline.time,'time_ns',return_value=0):
            try:pipeline.execute(configured,root/'resident-cancel-compilation',bridge,fake_prepared)
            except InterruptedError:pass
            else:raise AssertionError('compilation cancellation swallowed')
        assert actions==['preview_frozen','confirm','stop']
        # An already queued confirm is revoked even before automatic_enabled starts.
        actions.clear();owner_fields['automatic_enabled']=False
        with patch.object(pipeline,'resident_execution_identity',side_effect=InterruptedError('cancel queued confirm')), \
             patch.object(pipeline.time,'time_ns',return_value=0):
            try:pipeline.execute(configured,root/'resident-cancel-queued',bridge,fake_prepared)
            except InterruptedError:pass
            else:raise AssertionError('queued cancellation swallowed')
        assert actions==['preview_frozen','confirm','stop']
        owner_fields['automatic_enabled']=True
        # Execute's real Popen/cancel path reaps its local wrapper and child; no GPU.
        actions.clear();recording_fields['done']=False
        marker=root/'replay-child.pid';children=[]
        script='import os,time,pathlib;pathlib.Path('+repr(str(marker))+').write_text(str(os.getpid()));time.sleep(60)'
        argv=[sys.executable,str(Path(pipeline.__file__).with_name('owned_process.py')),'--timeout','60','--',sys.executable,'-c',script]
        def launch(*args,**kwargs):
            value=real_popen(*args,**kwargs);children.append(value);return value
        native.side_effect=lambda *args,**kwargs:dict(name='playground-replay',argv=argv,timeout_s=60)
        popen.side_effect=launch
        with patch.object(pipeline,'execution_runtime',return_value=runtime), \
             patch.object(pipeline,'prepare_remote_runtime',return_value='/remote/shared-runtime.json'), \
             patch.object(pipeline.time,'time_ns',return_value=0):
            try:pipeline.execute(configured,root/'resident-cancel-live-child',bridge,fake_prepared,cancel=marker.exists)
            except InterruptedError:pass
            else:raise AssertionError('live cancellation swallowed')
        assert children and all(child.poll() is not None for child in children)
        with real_popen(['ps','-o','stat=','-p',marker.read_text()],stdout=subprocess.PIPE,text=True) as probe:
            status=probe.communicate()[0].strip()
        assert not status or status.startswith('Z')
        assert actions==['preview_frozen','confirm','stop']
    print('PASS Playground pipeline: parent evidence, one confirm, shared30Hz runtime, independent capture/control outcomes and videos')


if __name__ == '__main__':
    main()
