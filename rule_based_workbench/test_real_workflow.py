"""Physical/fake state isolation and preview-only prepare; CPU, no robot/network."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from integration import _digest
from real_bridge import RealBridge
from real_workflow import load, summary, playback, live_preview, image_evidence, FILES
from test_frozen_executor import fixture


def main():
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
