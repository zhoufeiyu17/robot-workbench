"""Physical/fake state isolation and preview-only prepare; CPU, no robot/network."""
from copy import deepcopy
import hashlib
import json
import sys
import inspect
from pathlib import Path
from tempfile import TemporaryDirectory

from integration import _digest
from real_bridge import RealBridge
from real_workflow import load, summary, playback, live_preview, image_evidence, FILES
from test_frozen_executor import fixture


def check_disabled_workflow(source=None, plan=None, binding=None, *, mapping_path=None):
    """Artifact loading plus optional real shared compiler on a CPU backend; no SDK or sends."""
    from physical_motiongen import freeze_physical
    if source is None:
        from test_fixed_right_policy import fixture as fixed_fixture
        source,model,binding=fixed_fixture();plan=freeze_physical(model,source,binding)
    assert source['initial_state']['disabled_grippers']==['right']
    with TemporaryDirectory() as directory:
        root=Path(directory)
        config=dict(environment='physical',robot_id=source['robot_id'],control_url='http://127.0.0.1:1',
            cameras={camera:dict(serial=source['camera_serials'][side])
                     for camera,side in zip(('base_0','left_wrist_0','right_wrist_0'),('head','left','right'))})
        for name,value in (('source_path',source),('plan_path',plan),('hardware_binding',binding)):
            path=root/(name+'.json');path.write_text(json.dumps(value));config[name]=str(path)
        loaded=load(config)
        assert not loaded['blockers'] and loaded['plan']==plan, loaded['blockers']
        bridge=RealBridge.__new__(RealBridge);bridge.config=config
        requests=[]
        bridge.request=lambda url,payload,**kw:(requests.append(payload) or b'{}','application/json')
        bridge.control(dict(action='preview_frozen'))
        bridge.control(dict(action='confirm',token='fixture',confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        assert [row['action'] for row in requests]==['preview_frozen','confirm']
        assert requests[0]['source']==source and requests[0]['plan']==plan
        compiled=None
        if mapping_path is not None:
            repo=Path(__file__).resolve().parents[1]
            sys.path.insert(0,str(repo/'tests'))
            from test_owner_clock import ClockBackend,Safety,calibration,jaw
            from robot_workbench.runtime import Runtime
            from execution_timeline import validate
            from real_recording import RealRecorder
            assert hashlib.sha256(Path(mapping_path).read_bytes()).hexdigest()==source['calibration_profile']['model_binding']['jaw_mapping_sha256']
            jaw.configure_mapping(mapping_path)
            first=source['initial_state']['planned_reported'];clock=[10.]
            q=[*first['left_arm_joints'],jaw.feedback_sdk('left',first['left_gripper']),*first['right_arm_joints'],0.]
            backend=ClockBackend(lambda:clock[0],q);backend.passive_jaws=('right',)
            runtime=Runtime(calibration,Safety,lambda:backend,device_id='cpu-disabled-source-chain',environment='physical',
                lock_root=root/'lock',initial_holder='collection',control_clock=lambda:clock[0],jaw_mapping_file=mapping_path,
                hardware_binding=config['hardware_binding'],initialize_backend=lambda b:b.set_damping_hold(manual=True))
            try:
                credential=runtime.register_business('collection')
                for _ in range(42):clock[0]+=.01;runtime.tick()
                assert runtime.owner.backend.passive_jaws==('right',)
                before=list(backend.events)
                # Actual backend state still owns the runtime gate, regardless of source declaration.
                backend.passive_jaws=()
                try:runtime.submit(credential,dict(action='preview_frozen',source=source,plan=plan))
                except ValueError as error:assert 'disabled' in str(error)
                else:raise AssertionError('runtime accepted a source-only disabled declaration')
                backend.passive_jaws=('right',)
                review=runtime.submit(credential,dict(action='preview_frozen',source=source,plan=plan))
                assert review['disabled_grippers']==['right'] and review['held_arms']==['right']
                proxy=runtime.owner.pending[review['token']][0][0]
                payload,frozen,base=runtime.owner.frozen_payloads[proxy.sha256]
                runner_type=inspect.getclosurevars(runtime.collection.start_motion).nonlocals['Runner']
                runner=runner_type(runtime.owner,proxy,payload,frozen,base)
                from unittest.mock import patch
                from robot_workbench.control_plan import load_artifact
                with patch.object(runtime.owner,'runner',runner):
                    compiled=runner.compile_runtime(q)
                # This explicit CPU audit/recorder fixture is outside the Owner
                # loop; production keeps only the sealed rows and artifact ref.
                compiled=load_artifact(compiled['execution_timeline_ref'])
                assert compiled['initialization']['disabled_grippers']==['right']
                assert compiled['initialization']['held_arms']==['right']
                assert compiled['initialization']['actual_start']['right_gripper']==0.
                validate(compiled,plan)
                recorder=RealRecorder(root/'recording',source,plan,binding,execution_timeline=compiled)
                assert recorder.frozen.disabled_grippers==('right',)
                recorder.abort()
                # Queue only; never tick a confirmation or start an execution.
                result=runtime.submit(credential,dict(action='confirm',token=review['token'],
                    confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                assert result['queued']=='confirm' and backend.events==before
            finally:
                backend.close();runtime.close();jaw.configure_mapping(None)
        return dict(artifact_load_ready=True,bridge_preview_and_confirm_artifact_checks=True,
            actual_shared_compiler=compiled is not None,disabled_grippers=['right'],
            compiled_commands=len(compiled['commands']) if compiled else None,
            runtime_disabled_guard_preserved=compiled is not None,backend_motion_calls=0)


def main():
    check_disabled_workflow()
    source,plan = fixture(.001)
    real = {k:deepcopy(v) for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
    serials = dict(head='h',left='l',right='r')
    real.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',robot_id='arx6',camera_serials=serials)
    binding = dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='arx6',camera_serials=serials,
                   source_digest=_digest(real),calibration_profile_digest=_digest(real['calibration_profile']))
    from test_joint_space import physical_fixture
    physical_fixture(plan, real)
    config = dict(robot_id='arx6',cameras={key:dict(serial=serial) for key,serial in zip(('base_0','left_wrist_0','right_wrist_0'),serials.values())})
    empty = summary(config,{})
    assert empty['scene'] is None and empty['state'] is None and not empty['workflow']['ready']
    assert len(empty['workflow']['blockers']) >= 3
    with TemporaryDirectory() as directory:
        root = Path(directory)
        for key,value in [('source_path',real),('plan_path',plan),('hardware_binding',binding)]:
            path = root/(key+'.json'); path.write_text(json.dumps(value)); config[key] = str(path)
        config['artifact_dir'] = directory
        assert load(config)['plan'] == plan
        missing = dict(config); del missing['plan_path']
        assert load(missing)['source'] == real and load(missing)['plan'] is None
        measured = plan['ticks'][0]['target']
        fake_owner = dict(mode='simulation',physical_feedback=measured,frozen={'timeline_digest':plan['timeline_digest'],'status':'completed'})
        assert summary(config,fake_owner)['state'] is None
        assert summary(config,fake_owner)['pipeline']['execution'] == 'waiting'
        physical = dict(fake_owner,mode='hardware')
        assert summary(config,physical)['state'] == measured
        assert summary(config,physical)['pipeline']['execution'] == 'completed'
        assert summary(config,physical)['pipeline']['recording'] == {}, 'command completion is not raw acceptance'
        physical['frozen'].update(last_tick=2,status='failed',reason='tracking error')
        failed = summary(config,physical)
        assert failed['pipeline']['execution'] == 'failed'
        assert failed['telemetry']['source']['failure']['event'] == 'tracking error'
        assert not failed['pipeline']['recording']
        from live_preview import write_frame
        import time
        metadata = dict(source_run_id=real['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],
                        camera='base_0',evidence_source='simulation',published_at_ns=time.time_ns())
        frame_path = root/'replay-target/live-preview.frame'
        write_frame(frame_path,b'\xff\xd8fixture',metadata)
        assert live_preview(config,'target')[0].payload == b'\xff\xd8fixture'
        assert live_preview(config,'source') is None
        write_frame(frame_path,b'\xff\xd8fixture',dict(metadata,source_run_id='old-fake-run'))
        assert live_preview(config,'target') is None
        image = b'\x89PNG\r\n\x1a\nfixture'; (root/'source.png').write_bytes(image)
        evidence = dict(schema_version='physical_collection_evidence.v1',images={'source.png':dict(
            path='source.png',source_run_id=real['run_id'],source_sequence=real['sequence'],
            scene_digest=real['scene']['scene_digest'],evidence_source='physical_rgbd',sha256=hashlib.sha256(image).hexdigest())})
        (root/'evidence.json').write_text(json.dumps(evidence))
        assert image_evidence(config,'source.png') == (root/'source.png').resolve()
        evidence['images']['source.png']['evidence_source'] = 'simulation'
        (root/'evidence.json').write_text(json.dumps(evidence))
        assert image_evidence(config,'source.png') is None
        assert playback(config)[0]['status'] == 'unavailable'
        replay = root/'playback'; replay.mkdir()
        files = {}
        for name in FILES:
            value = b'0000ftyp-test'; (replay/name).write_bytes(value)
            files[name] = dict(size_bytes=len(value),sha256=hashlib.sha256(value).hexdigest())
        manifest = dict(schema_version='workbench_playback.v1',clock='simulation',files=files)
        (replay/'manifest.json').write_text(json.dumps(manifest))
        assert playback(config)[0]['status'] == 'unavailable', 'dual DSW replay must stay hidden'
        manifest.update(schema_version='physical_workbench_playback.v1',mode='real_robot',clock='elapsed',
                        source_run_id=real['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],
                        provenance=dict(source='physical_rgbd',target='simulation'))
        (replay/'manifest.json').write_text(json.dumps(manifest))
        assert playback(config)[0]['status'] == 'ready'
        (replay/next(iter(FILES))).write_bytes(b'0000ftyp-evil')
        assert playback(config)[0]['status'] == 'unavailable'
        (root/'source_path.json').write_text(json.dumps(source))
        assert load(config)['source'] is None, 'fake source must not be promoted'
    disabled = RealBridge.__new__(RealBridge)
    disabled.config = dict(config,hardware_access_enabled=False)
    assert disabled.status()['hardware_access_enabled'] is False
    assert not disabled.status()['collection_ready'] and not disabled.owner()['connected']
    for action in ('acquire','stop','recover','release','preview_frozen','confirm'):
        try: disabled.control({'action':action})
        except ValueError as error: assert '标定工作台' in str(error)
        else: raise AssertionError('disabled hardware mutation reached service')
    try: disabled.camera_stream('base_0')
    except ValueError: pass
    else: raise AssertionError('disabled camera stream reached service')
    bridge = RealBridge.__new__(RealBridge)
    bridge.config = {'environment':'physical'}
    calls = []
    bridge.status = lambda:dict(collection_ready=True)
    bridge.control = lambda value:calls.append(value) or {'token':'review'}
    assert bridge.prepare_collection()['status'] == 'awaiting-physical-confirmation'
    assert calls == [{'action':'preview_frozen'}], 'prepare must not confirm/acquire/move'
    bridge.status = lambda:dict(collection_ready=False,blockers=['缺少真实标定'])
    try: bridge.prepare_collection()
    except ValueError as error: assert '真实标定' in str(error)
    else: raise AssertionError('blocked prepare accepted')
    print('real workflow: physical artifact isolation, missing-stage status, replay integrity and preview-only prepare PASS')


if __name__ == '__main__': main()
