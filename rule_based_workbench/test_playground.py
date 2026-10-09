"""CPU-only fake bridge/pipeline checks; never contacts cameras, DSW or motors."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

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


def main():
    executed = []

    def plan(config, output, bridge, speed, progress=None):
        progress('碰撞检查', estimate_s=2, stages=[dict(label='碰撞检查', status='completed', elapsed_s=.1, estimate_s=2)])
        files = {}
        for key in ('source_path', 'plan_path', 'hardware_binding'):
            path = output / (key + '.json')
            path.write_text('{}')
            files[key] = str(path)
        return dict(validated=True, timeline_digest='checked', **files)

    def execute(config, output, bridge, plan, progress=None):
        for key in ('errors', 'recording', 'simulation', 'physical_outcome', 'task_outcome'):
            assert key not in service.state, f'previous attempt leaked: {key}'
        assert service.state['plan']['timeline_digest'] == plan['timeline_digest']
        current_errors = {'controller': 'current attempt tracking failure'}
        progress('核验真机视频后等待独立仿真视频封存', errors=current_errors,
                 recording={'directory': str(output/'raw')}, physical_outcome='failed', task_outcome='unverified')
        current_errors['controller'] = 'mutated after callback'
        live = service.snapshot()
        assert live['busy'] and live['physical_outcome'] == 'failed'
        assert live['error'] == 'controller: current attempt tracking failure'
        assert live['message'].startswith('真机执行未完成；')
        assert live['can_execute'] is False
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
        service.start({'action': 'prepare'})
        service.thread.join(3)
        assert service.state['progress']['estimate_s'] == previous_prepare
        assert service.state['progress']['estimate_source'] == '上次成功耗时'
        assert len(bridge.commands) == 1, 'preparation timing must not reconnect an existing owner'
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
        service.state.update(errors={'controller': 'old attempt'}, recording={'directory': '/old/raw'},
                             simulation={'status': 'old'}, physical_outcome='failed', task_outcome='old')
        service.start({'action': 'execute', 'speed_scale': .25})
        service.thread.join(3)
        assert executed[0]['speed_scale'] == .25
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
    for sent_count in (0,3,None):
        counts={'plan':0,'execute':0}
        def counted_plan(config,output,bridge,speed,progress=None):
            counts['plan']+=1
            return plan(config,output,bridge,speed,progress=progress)
        def failed_execute(config,output,bridge,descriptor,progress=None):
            counts['execute']+=1
            checkpoint=json.loads((output.parents[1]/'prepared-plan.json').read_text())
            assert checkpoint['execution_pending'] and checkpoint['execution_output']==str(output)
            if sent_count is None:raise RuntimeError('connection lost; execution unknown')
            return dict(status='failed',videos=[],recording={'manifest':{'command_count':sent_count}})
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
            assert restarted.snapshot()['environment']['ready'] and not restarted.snapshot()['can_execute']
            restarted.start({'action':'prepare'});restarted.thread.join(3)
            assert restarted.snapshot()['can_execute'] and counts=={'plan':1,'execute':0}
            assert checkpoint.read_bytes()==before, 'prepare replaced the frozen checkpoint'
            restarted.start({'action':'execute'});restarted.thread.join(3)
            assert counts=={'plan':1,'execute':1}
            assert json.loads(checkpoint.read_text())['execution_pending'] is (sent_count!=0)
            assert restarted.snapshot()['can_execute'] is (sent_count==0)
            after=Playground(tmp,bridge,lambda:{'status':'ready'})
            after.start({'action':'prepare'});after.thread.join(3)
            assert after.snapshot()['can_execute'] is (sent_count==0)
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
