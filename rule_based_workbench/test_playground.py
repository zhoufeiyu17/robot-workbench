"""CPU-only fake bridge/pipeline checks; never contacts cameras, DSW or motors."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock, patch

from playground import Playground


class Bridge:
    def __init__(self):
        self.config = {'playground': {'enabled': True}}
        self.connected = False
        self.commands = []
        self.pedal = False

    def status(self):
        return {'connected': self.connected, 'owner': dict(mode='hardware', feedback_healthy=True,
                state='READY', stationary=True, automatic_enabled=False, recording_configured=True,
                pedal={'input_healthy': True, 'physically_down': self.pedal}),
                'cameras': {key: {'ready': True} for key in ('base_0', 'left_wrist_0', 'right_wrist_0')}}

    def control(self, command):
        self.commands.append(command)
        self.connected = True


def rejected(fn):
    try:
        fn()
    except ValueError:
        return
    raise AssertionError('request should have been rejected')


def in_process_execution_fixture(config,output,bridge_config,plan,progress=None,cancel=None):
    # UI state tests stub the process boundary; the spawn/HTTP suite tests its implementation.
    import sys
    return sys.modules['playground_pipeline'].execute(config,output,SimpleNamespace(config=bridge_config),plan,progress=progress)


@patch('playground_execution_process.execute',in_process_execution_fixture)
def main():
    executed = []

    # Acceptance publishes the real task identity before its worker completes.
    with TemporaryDirectory() as tmp:
        bridge=Bridge();bridge.connected=True
        service=Playground(tmp,bridge,lambda:{'status':'ready'})
        identities=[]
        for _ in range(2):
            entered=Event();release=Event()
            def blocked_run(*args):
                entered.set()
                assert release.wait(2), 'test worker not released'
            with patch.object(service,'run',side_effect=blocked_run):
                try:
                    receipt=service.start({'action':'prepare'})
                    assert entered.wait(1)
                    state=service.snapshot();saved=json.loads((Path(tmp)/'state.json').read_text())
                    assert receipt['action']==state['action']=='prepare'
                    assert receipt['output']==state['output']==saved['output']
                    assert state['busy'] and state['status']=='running'
                    assert Path(receipt['output']).is_dir()
                    identities.append(receipt['output'])
                finally:release.set();service.thread.join(2)
            with service.lock:service.state['busy']=False
        assert identities[0]!=identities[1]
        health=bridge.status();health['cameras']['base_0']['ready']=False
        with patch.object(bridge,'status',return_value=health):
            assert service.snapshot()['capture_warning']=='图像或录制状态待确认；动作独立运行，数据完整性以封存校验为准。'
            service.state['error']='recording: actual recorder failure'
            assert service.snapshot()['error']=='recording: actual recorder failure'

    def plan(config, output, bridge, speed, progress=None):
        progress('碰撞检查', estimate_s=2, stages=[dict(label='碰撞检查', status='completed', elapsed_s=.1, estimate_s=2)])
        files = {}
        for key in ('source_path', 'plan_path', 'hardware_binding'):
            path = output / (key + '.json')
            path.write_text('{}')
            files[key] = str(path)
        return dict(validated=True, timeline_digest='checked', **files)

    def execute(config, output, bridge, plan, progress=None):
        for key in ('errors', 'recording', 'simulation', 'physical_outcome', 'task_outcome', 'dataset'):
            assert key not in service.state, f'previous attempt leaked: {key}'
        assert service.state['plan']['timeline_digest'] == plan['timeline_digest']
        current_errors = {'controller': 'current attempt tracking failure'}
        progress('核验真机视频后等待独立仿真视频封存', errors=current_errors,
                 recording={'directory': str(output/'raw')}, physical_outcome='failed', task_outcome='unverified',
                 dataset={'dataset_ready':False,'error':'current readback failure'})
        current_errors['controller'] = 'mutated after callback'
        live = service.snapshot()
        assert live['busy'] and live['physical_outcome'] == 'failed'
        assert live['error'] == 'controller: current attempt tracking failure'
        assert live['message'].startswith('真机执行未完成；')
        assert live['can_execute'] is False
        assert live['dataset']['error']=='current readback failure' and 'dataset'not in live['progress']
        assert 'simulation' not in live and 'errors' not in live['progress']
        assert json.loads((service.directory/'state.json').read_text())['errors']['controller'] == 'current attempt tracking failure'
        executed.append(deepcopy(plan))
        video = output / 'source.mp4'
        video.write_bytes(b'fake-video-bytes')
        return dict(status='stopped', message='fake pedal stop', videos=[dict(label='真机', path=str(video))])

    def restore(config,directory,bridge):
        path=directory/'prepared-plan.json'
        if not path.is_file():return None
        checkpoint=json.loads(path.read_text())
        return None if checkpoint['execution_pending'] else checkpoint['plan']

    fake = SimpleNamespace(prepare_environment=lambda config: [dict(key='dsw', status='ready'),
                                                               dict(key='simulation', status='ready')],
                           plan=plan, execute=execute,restore_plan=restore)
    with TemporaryDirectory() as tmp, patch.dict('sys.modules',playground_pipeline=fake):
        bridge=Bridge();service=Playground(tmp,bridge,lambda:{'status':'ready'})
        error=RuntimeError('scene setup failed')
        error.preparation_failure=dict(stage='scene_setup',native_status='failed',native_phase='setup',
            native_reason='reset characterization rejected: cup_table_recontact_or_invalid_support')
        service.prepare_services=Mock(side_effect=error)
        service.start({'action':'prepare','arms_supported':True});service.thread.join(3)
        assert service.state['warmup']['failure']==error.preparation_failure
        assert not service.state['busy'] and not service.state['environment']['ready']
        service.prepare_services.assert_called_once();assert bridge.commands==[], 'failure must not retry or acquire control'
    with TemporaryDirectory() as tmp, patch.dict('sys.modules', playground_pipeline=fake):
        bridge = Bridge()
        service = Playground(tmp, bridge, lambda: {'status': 'ready'})
        assert service.snapshot()['can_execute'] is False
        assert bridge.commands == []
        rejected(lambda: service.start({'action': 'prepare'}))
        rejected(lambda: service.start({'action': 'execute'}))
        rejected(lambda: service.start({'action': 'plan', 'speed_scale': float('nan')}))
        service.start({'action': 'prepare', 'arms_supported': True})
        service.thread.join(3)
        assert not service.thread.is_alive()
        assert service.snapshot()['environment']['ready'] is True, service.state
        assert service.state['progress']['estimate_s'] == 180
        assert '粗估' in service.state['progress']['estimate_source']
        assert service.state['timing_history']['prepare'] >= 0
        assert bridge.commands == [{'action': 'acquire', 'confirmation': 'ARMS_SUPPORTED_CONNECT'}]
        assert service.snapshot()['can_execute'] is False
        previous_prepare = service.state['timing_history']['prepare']
        previous_output=service.state['output']
        assert service.start({'action': 'prepare'})['reused'] is True
        service.thread.join(3)
        assert service.state['output']==previous_output
        assert service.state['timing_history']['prepare']==previous_prepare
        assert len(bridge.commands) == 1, 'preparation timing must not reconnect an existing owner'
        initialization=service.initialization
        service.initialization=lambda:{'status':'ready','planner':{'status':'ready','model_initialized':True},
            'scene':{'status':'stopped','model_initialized':False}}
        assert service.snapshot()['warmup']['status']=='invalidated'
        assert not service.snapshot()['environment']['ready']
        service.initialization=initialization
        service.start({'action': 'plan', 'speed_scale': .3})
        service.thread.join(3)
        assert service.snapshot()['can_execute'] is True, service.state
        assert service.state['plan']['first_frame_url'] is None
        assert service.state['progress']['next_label'] == '核对首帧后点击 action 真机执行'
        assert executed == [], 'planning never executes'
        bridge.pedal = True
        assert service.snapshot()['can_execute'] is False
        rejected(lambda: service.start({'action': 'execute'}))
        bridge.pedal = False
        service.state.update(errors={'controller': 'old attempt'}, recording={'directory': '/old/raw'},dataset={'dataset_ready':True,'dataset_root':'/old/dataset'},
                             simulation={'status': 'old'}, physical_outcome='failed', task_outcome='old')
        for invalid in (.29,1.71,True,float('nan')):
            rejected(lambda: service.start({'action':'execute','speed_scale':invalid}))
        service.start({'action': 'execute', 'speed_scale': 1.7})
        service.thread.join(3)
        assert executed[0]['speed_scale'] == 1.7
        assert service.state['status'] == 'stopped'
        assert service.snapshot()['can_execute'] is False, 'a submitted plan is consumed'
        assert service.media('video-0') is None, 'media revision required'
        revision = service.state['videos'][0]['url'].rsplit('v=', 1)[1]
        assert service.media('video-0', revision).read_bytes() == b'fake-video-bytes'
        assert service.media('../state.json', revision) is None
        restarted = Playground(tmp, bridge, lambda: {'status': 'ready'})
        assert restarted.snapshot()['can_execute'] is False
        assert restarted.state['plan']['validated'] is False
        assert len(restarted.state['videos']) == 1
        assert json.loads((Path(tmp) / 'state.json').read_text())['status'] == 'stopped'
        rejected(lambda: service.artifact('/etc/hosts'))
        import app
        from test_app import serving
        with patch.object(app, 'PLAYGROUND', service), serving(app.WorkbenchState()) as request:
            state = request('GET', '/api/playground')
            assert not state['can_execute']
            request('POST', '/api/playground', {'action': 'execute'}, expected=403)
            request('POST', '/api/playground', {'action': 'execute'}, expected=400,
                    headers={'X-Workbench-CSRF': app.CSRF_TOKEN})
            video = request('GET', '/api/playground/media/video-0?v=' + revision,
                            expected=206, headers={'Range': 'bytes=0-3'})
            assert video == b'fake'
            request('GET', '/api/playground/media/video-0?v=stale', expected=404)
            request('GET', '/api/playground/media/../state.json?v=' + revision, expected=404)
            service.state['busy'] = True
            blocked = request('POST', '/api/mode', {'mode': 'real'}, expected=400,
                              headers={'X-Workbench-CSRF': app.CSRF_TOKEN})
            assert 'Playground' in blocked['error']
            with patch.object(app, 'WORKBENCH_MODE', 'real'), patch.object(app, 'REAL_BRIDGE', bridge):
                request('POST', '/api/real/control', {'action': 'stop'},
                        headers={'X-Workbench-CSRF': app.CSRF_TOKEN})
                assert bridge.commands[-1] == {'action': 'stop'}, 'busy gate must retain stop'
            service.state['busy'] = False
        bridge.config = {}
        rejected(lambda: service.start({'action': 'plan'}))
        assert service.snapshot()['can_execute'] is False
    # A restart/prepare reuses an approved plan without planning or automatic motion.
    for outcome,sent_count in (('completed',0),('unknown',0),('unknown',3),('unknown',None),('pre_action',None)):
        counts={'plan':0,'execute':0}
        def counted_plan(config,output,bridge,speed,progress=None):
            counts['plan']+=1
            return plan(config,output,bridge,speed,progress=progress)
        def failed_execute(config,output,bridge,descriptor,progress=None):
            counts['execute']+=1
            checkpoint=json.loads((output.parents[1]/'prepared-plan.json').read_text())
            assert checkpoint['execution_pending'] and checkpoint['execution_output']==str(output)
            (output/'execution-phase.json').write_text(json.dumps(dict(
                phase='pre_action' if outcome=='pre_action' else 'confirm_requested',timeline_digest=descriptor['timeline_digest'])))
            if sent_count is None:raise RuntimeError('connection lost; execution unknown')
            return dict(status='failed',videos=[],physical_outcome=outcome,
                recording={'manifest':{'command_count':sent_count,'controller_outcome':outcome,
                    'camera_frames':dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'),0)}})
        candidate=SimpleNamespace(prepare_environment=fake.prepare_environment,plan=counted_plan,
                                  execute=failed_execute,restore_plan=restore)
        with TemporaryDirectory() as tmp,patch.dict('sys.modules',playground_pipeline=candidate):
            bridge=Bridge();bridge.connected=True
            service=Playground(tmp,bridge,lambda:{'status':'ready'})
            service.start({'action':'prepare'});service.thread.join(3)
            service.start({'action':'plan'});service.thread.join(3)
            checkpoint=Path(tmp)/'prepared-plan.json'
            assert json.loads(checkpoint.read_text())['execution_pending'] is False
            before=checkpoint.read_bytes()
            restarted=Playground(tmp,bridge,lambda:{'status':'ready'})
            assert not restarted.snapshot()['environment']['ready'] and not restarted.snapshot()['can_execute']
            assert restarted.snapshot()['warmup']['status']=='invalidated'
            restarted.start({'action':'prepare'});restarted.thread.join(3)
            assert restarted.snapshot()['can_execute'] and counts=={'plan':1,'execute':0}
            assert checkpoint.read_bytes()==before, 'prepare replaced the frozen checkpoint'
            restarted.start({'action':'execute'});restarted.thread.join(3)
            assert counts=={'plan':1,'execute':1}
            assert json.loads(checkpoint.read_text())['execution_pending'] is (outcome!='pre_action')
            assert restarted.snapshot()['can_execute'] is (outcome=='pre_action')
            after=Playground(tmp,bridge,lambda:{'status':'ready'})
            after.start({'action':'prepare'});after.thread.join(3)
            assert after.snapshot()['can_execute'] is (outcome=='pre_action')
            assert counts=={'plan':1,'execute':1}, 'prepare must not replan or replay actions'
            assert bridge.commands==[], 'recovery should not reconnect an existing owner'
    # Stage clocks reset independently; repeated updates and snapshots do not reset them.
    with TemporaryDirectory() as tmp, patch('playground.time.monotonic') as clock:
        bridge = Bridge()
        service = Playground(tmp, bridge, lambda: {'status': 'ready'})
        service.started = service.stage_started = 10
        service.state['busy'] = True
        clock.return_value = 20
        service.progress('检测', stages=[dict(label='检测', status='running', elapsed_s=0)])
        clock.return_value = 24
        state = service.snapshot()['progress']
        assert state['elapsed_s'] == 14 and state['stage_elapsed_s'] == 4
        assert state['stages'][0]['elapsed_s'] == 4
        clock.return_value = 25
        service.progress('检测', stage_status='completed')
        clock.return_value = 30
        assert service.snapshot()['progress']['stage_elapsed_s'] == 5
        service.progress('规划')
        assert service.snapshot()['progress']['stage_elapsed_s'] == 0
        assert service.snapshot()['progress']['elapsed_s'] == 20
    print('PASS Playground fake prepare/plan/execute, no auto motion, pedal/state gates, speed, consumption, restart and media bounds')


if __name__ == '__main__':
    main()
