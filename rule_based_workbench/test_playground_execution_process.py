"""Spawned real pipeline and loopback HTTP fixtures; no SDK, GPU, SSH, or robot commands."""
from copy import deepcopy
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from threading import Event,Thread
import time
from unittest.mock import patch

import playground_execution_process as isolated


def fixture_worker(*args):
    """Patch only remote replay/config IO; retain actual execute/validators/RealBridge HTTP."""
    import playground_pipeline as pipeline
    config=args[3];cancelled=args[1]
    if config['test_mode']=='pre_confirm_cancel':
        def acknowledge_cancel():
            if cancelled.wait(10):
                (Path(args[4])/'test-cancel-observed').write_text('shared cancellation observed')
        Thread(target=acknowledge_cancel,daemon=True).start()
    def runtime(*unused,**kwargs):
        if config['test_mode']=='crash':os._exit(17)
        if config['test_mode']=='cancel':
            while not cancelled.is_set():time.sleep(.01)
            raise InterruptedError('CPU fixture cancellation after confirm')
        raise ValueError('CPU fixture has no simulation replay or camera recording')
    with patch.object(pipeline,'load_config',return_value=(dict(environment=args[5].get('environment','physical'),remote_run_root='/cpu/frozen'),{},{})), \
         patch.object(pipeline,'execution_runtime',side_effect=runtime):
        isolated._worker(*args)


def check(artifacts=None,*,environment='physical'):
    if environment not in ('physical','fake'):raise ValueError('explicit test environment required')
    if artifacts is None:
        if environment=='physical':from test_fixed_right_policy import fixture
        else:from test_fake_device_pipeline import fake_inputs as fixture
        from physical_motiongen import freeze_physical
        source,model,binding=fixture();model['planning_validation']['derivation']=dict(speed_scale=.3)
        plan=freeze_physical(model,source,binding,environment=environment)
    else:
        read=lambda name:json.loads((Path(artifacts)/name).read_text())
        source,plan,binding=read('source.json'),read('plan.json'),read('hardware-binding.json')
    beats=[];done=Event()
    def beat():
        while not done.wait(.005):beats.append(time.monotonic())
    thread=Thread(target=beat);thread.start()
    try:
        with TemporaryDirectory() as directory:
            root=Path(directory);token=root/'token';token.write_text('x'*48)
            for name,value in (('source',source),('plan',plan),('binding',binding)):
                (root/(name+'.json')).write_text(json.dumps(value))
            events=[];confirmed=Event();previewed=Event();mode=['complete'];state={}
            def reset():
                state.clear();state.update(pid=4242,runtime_nonce='cpu-owner',command_epoch=7,holder='collection',
                    execution_instance_id='before',frozen={},motion={},mode='simulation' if environment=='fake' else 'hardware',
                    environment=environment,evidence_source='fake_device' if environment=='fake' else 'physical_hardware',
                    simulation_only=environment=='fake',
                    connected=True,feedback_healthy=True,stationary=True,state='READY',automatic_enabled=False,recording=None)
                events.clear();confirmed.clear();previewed.clear()
            class Handler(BaseHTTPRequestHandler):
                def log_message(self,*unused):pass
                def reply(self,value):
                    raw=json.dumps(value).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers()
                    try:self.wfile.write(raw)
                    except (BrokenPipeError,ConnectionResetError):pass
                def do_GET(self):self.reply(deepcopy(state))
                def do_POST(self):
                    value=json.loads(self.rfile.read(int(self.headers['Content-Length'])));action=value['action'];events.append(action)
                    if action=='preview_frozen':
                        assert value['plan']==plan and value['source']==source
                        previewed.set()
                        if mode[0]=='pre_confirm_cancel':
                            # Hold the mock response until the child's shared cancellation
                            # is observed, not merely requested by the parent test thread.
                            acknowledged=output/'test-cancel-observed';deadline=time.monotonic()+10
                            while not acknowledged.is_file() and time.monotonic()<deadline:time.sleep(.005)
                            assert acknowledged.is_file(),'worker did not observe cancellation before preview returned'
                        self.reply(dict(token='one-review',timeline_digest=plan['timeline_digest'],
                            owner_pid=4242,command_epoch=7,compiled_tick_count_before_initialization=len(plan['ticks'])))
                    elif action=='confirm':
                        state.update(execution_instance_id='current',automatic_enabled=mode[0]!='complete',
                            frozen=dict(timeline_digest=plan['timeline_digest'],status='completed' if mode[0]=='complete' else 'running'),
                            motion=dict(plan_sha256='cpu-proxy'))
                        confirmed.set()
                        if mode[0]=='lost_reply':time.sleep(2.3)
                        self.reply(dict(queued='confirm',plan_sha256='cpu-proxy'))
                    elif action=='stop':
                        state.update(command_epoch=8,automatic_enabled=False);self.reply(dict(queued='stop',command_epoch=8))
                    else:raise AssertionError(action)
            server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.daemon_threads=True
            server_thread=Thread(target=server.serve_forever,daemon=True);server_thread.start()
            url=f'http://127.0.0.1:{server.server_port}'
            bridge=dict(environment=environment,control_url=url,token_file=str(token),robot_id=source['robot_id'],
                cameras={camera:dict(url=url,serial=source['camera_serials'][side]) for camera,side in
                         zip(('base_0','left_wrist_0','right_wrist_0'),('head','left','right'))},
                trusted_binding_path=str(root/'binding.json'),local_recording_root=str(root/'raw'))
            prepared=dict(config_path=str(root/'config.json'),plan_path=str(root/'plan.json'),source_path=str(root/'source.json'),
                hardware_binding=str(root/'binding.json'),speed_scale=.3,timeline_digest=plan['timeline_digest'],environment=environment)
            try:
                for kind in ('complete','cancel','crash','pre_confirm_cancel','lost_reply'):
                    reset();mode[0]=kind;cancel=Event();output=root/kind
                    trigger=None
                    if kind in ('cancel','pre_confirm_cancel'):
                        def trigger_cancel(event=confirmed if kind=='cancel' else previewed):
                            assert event.wait(20);cancel.set()
                        trigger=Thread(target=trigger_cancel);trigger.start()
                    with patch.object(isolated,'_worker',fixture_worker):
                        try:
                            result=isolated.execute(dict(test_mode=kind),output,bridge,prepared,cancel=cancel.is_set)
                            assert kind=='complete' and result['status']=='failed' # No simulated recording was fabricated.
                        except (InterruptedError,RuntimeError):assert kind!='complete'
                    if trigger:trigger.join(2)
                    if kind=='pre_confirm_cancel':
                        assert (output/'test-cancel-observed').is_file()
                        assert 'confirm' not in events and 'stop' not in events,events
                    else:assert events.count('confirm')==1,events
                    if kind in ('cancel','crash','lost_reply'):
                        assert events.count('stop')==1,events
                        receipt=json.loads((output/'execution-stop-request.json').read_text())
                        assert receipt['status']=='requested' and receipt['physical_stop']=='unverified'
                    identity=json.loads((output/'execution-worker.json').read_text())
                    assert identity['start_method']=='spawn' and identity['pid']!=os.getpid()
                # Pure protocol case: a queued confirm has not changed the old instance.
                from playground_pipeline import stop_physical_execution,save
                from real_bridge import RealBridge
                reset();queued=root/'queued';queued.mkdir()
                save(queued/'execution-phase.json',dict(phase='confirm_requested'))
                save(queued/'execution-control-identity.json',dict(owner_pid=4242,command_epoch=7,runtime_nonce='cpu-owner',holder='collection',
                    preview_owner_pid=4242,preview_command_epoch=7,
                    before_execution_instance_id='before',before_frozen={},before_motion={},timeline_digest=plan['timeline_digest'],preview_token_sha256='proof'))
                lane=RealBridge(bridge)
                a=stop_physical_execution(lane,queued);b=stop_physical_execution(lane,queued)
                assert a==b and events==['stop'] and a['physical_stop']=='unverified'
                # Owner replacement must not receive an old task's stop.
                reset();state['pid']=4243;other=root/'other';other.mkdir()
                for name in ('execution-phase.json','execution-control-identity.json'):(other/name).write_bytes((queued/name).read_bytes())
                assert stop_physical_execution(lane,other)['status']=='unknown' and not events
            finally:server.shutdown();server.server_close();server_thread.join(2)
    finally:done.set();thread.join(2)
    largest=max(b-a for a,b in zip(beats,beats[1:]))
    assert largest<.1,largest
    return dict(environment=environment,ticks=len(plan['ticks']),max_parent_heartbeat_gap_s=largest,limit_s=.1,
                spawned_pipeline=True,confirm_retries=0,pre_confirm_cancel_commands=0,
                after_confirm_cancel_stop_count=1,crashed_worker_stop_count=1,queued_confirmation_revoked=True,
                lost_confirm_reply_stop_count=1,changed_owner_untouched=True,robot_or_gpu_used=False)


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path,nargs='?')
    parser.add_argument('--environment',choices=('physical','fake'),default='physical')
    args=parser.parse_args()
    print(json.dumps(check(args.root,environment=args.environment),indent=2))
