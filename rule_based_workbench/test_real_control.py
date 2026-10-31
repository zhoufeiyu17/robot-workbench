"""CPU-only owner protocol check; no SDK initialization or network."""
from copy import deepcopy
from enum import Enum
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys

from real_control import load_calibration, classes, Lifecycle
from test_frozen_executor import fixture


class State(Enum):
    READY='READY'
    STOPPED='STOPPED_DAMPING_HOLD'
    MANUAL='MANUAL_TAKEOVER'
    FAULT='FAULT'
    RELEASED='RELEASED'


class Takeover:
    def __init__(self, backend, **kwargs):
        self.backend, self.state, self.reason = backend, State.READY, ''
        self.auto_aborted = False
    def fault(self, reason):
        self.state, self.reason = State.FAULT, reason
        self.backend.set_damping_hold()
    def press(self):
        self.state = State.STOPPED
        self.backend.set_damping_hold()
    def release(self):
        self.backend.close()
        self.state = State.RELEASED


def main(root, jaw_mapping_file=None):
    from concurrent.futures import ThreadPoolExecutor
    acquire = dict(action='acquire',confirmation='ARMS_SUPPORTED_CONNECT')
    lifecycle = Lifecycle()
    def request(_):
        try: lifecycle.submit(acquire); return True
        except ValueError: return False
    with ThreadPoolExecutor(max_workers=2) as pool:
        assert sum(pool.map(request, range(2))) == 1
    lifecycle.submit({'action':'stop'})
    assert not lifecycle.take_acquisition()
    lifecycle.submit(acquire)
    assert lifecycle.take_acquisition()
    lifecycle.submit({'action':'stop'})
    assert not lifecycle.install(object()), 'cancelled acquisition must not install owner'
    lifecycle.finish_acquisition()
    lifecycle.signal()
    assert lifecycle.exit_requested and lifecycle.owner is None
    lifecycle = Lifecycle()
    commands = []
    fake = SimpleNamespace(submit=lambda command: commands.append(command),snapshot=lambda:{})
    lifecycle.submit(acquire)
    assert lifecycle.take_acquisition() and lifecycle.install(fake)
    lifecycle.submit({'action':'stop'})
    lifecycle.finish_acquisition()
    assert lifecycle.stop_latched and commands[-1] == {'action':'stop'}
    lifecycle.signal()
    assert lifecycle.snapshot(True)['shutdown_pending']
    # Block expensive preview work while lifecycle polling and stop/replacement proceed.
    import threading
    for race in ('none','replace','stop'):
        started,release=threading.Event(),threading.Event()
        class PreviewOwner:
            def __init__(self):self.lock=threading.RLock();self.epoch=0;self.pending={}
            def submit(self,command):
                if command['action']=='stop':
                    with self.lock:self.epoch+=1
                    return {'queued':'stop'}
                started.set()
                if not release.wait(2):raise AssertionError('test preview was not released')
                with self.lock:self.pending['fixture-token']=True
                return {'token':'fixture-token'}
        lifecycle=Lifecycle();current=PreviewOwner();lifecycle.owner=current
        with ThreadPoolExecutor(max_workers=2) as pool:
            preview=pool.submit(lifecycle.submit,{'action':'preview_frozen'})
            try:
                assert started.wait(1)
                assert pool.submit(lifecycle.take_acquisition).result(timeout=.5) is False
                if race=='replace':
                    assert pool.submit(lifecycle.install,PreviewOwner()).result(timeout=.5)
                elif race=='stop':
                    assert pool.submit(lifecycle.submit,{'action':'stop'}).result(timeout=.5)=={'queued':'stop'}
            finally:release.set()
            if race=='none':assert preview.result(timeout=1)['token']=='fixture-token'
            else:
                try:preview.result(timeout=1)
                except ValueError as error:assert 'changed during preview' in str(error)
                else:raise AssertionError('stale owner preview escaped lifecycle recheck')
                assert 'fixture-token' not in current.pending
    ref = load_calibration(root)
    Owner, sdk, physical = classes(ref, jaw_mapping_file=jaw_mapping_file)
    from arx_env.joint_head_jaw_mapping import FEEDBACK, GAP_MM, interpolate
    source, plan = fixture(.001)
    plan['stages'] = [dict(name='initial_open', start_tick=0)]
    from real_contract import validate_real_plan
    from integration import _digest
    if jaw_mapping_file:
        from jaw_mapping_v2 import FEEDBACK, GAP_MM, MAPPING_ID
        from test_joint_space import a1_profile
        from joint_space import convert_state
        source['calibration_profile'] = a1_profile(source['calibration_profile'])
        import hashlib
        source['calibration_profile']['model_binding']['jaw_mapping_sha256'] = hashlib.sha256(Path(jaw_mapping_file).read_bytes()).hexdigest()
        plan['calibration_profile_digest'] = _digest(source['calibration_profile'])
        for tick in plan['ticks']:
            tick['target']['left_gripper'] = tick['target']['right_gripper'] = 82.992
        plan['timeline_digest'] = _digest(plan['ticks'])
        target = sdk(convert_state(plan['ticks'][0]['target'], source['calibration_profile'], to_reported=True))
        assert abs(target[6] - .077815) < 1e-12
        assert abs(target[13] - .07633375) < 1e-12
    real = {k:deepcopy(v) for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
    real.update(schema_version='physical_collection_source.v1', evidence_source='physical_rgbd',
                robot_id='arx6-test', camera_serials=dict(head='h',left='l',right='r'))
    binding = dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=real['robot_id'],
                   camera_serials=real['camera_serials'],calibration_profile_digest=_digest(real['calibration_profile']),
                   source_digest=_digest(real))
    from test_joint_space import physical_fixture
    physical_fixture(plan, real)
    validate_real_plan(plan, real, binding)
    for field in ('robot_id','source_digest','calibration_profile_digest','calibration_accepted'):
        bad = dict(binding, **{field:None})
        try: validate_real_plan(plan, real, bad)
        except ValueError: pass
        else: raise AssertionError('invalid physical binding accepted: '+field)
    try: validate_real_plan(plan, source, binding)
    except ValueError: pass
    else: raise AssertionError('simulation provenance accepted')
    def raw(target):
        values = sdk(target)
        for side,index,key in [('left',6,'left_gripper'),('right',13,'right_gripper')]:
            values[index] = interpolate(target[key], GAP_MM[side], FEEDBACK[side])
        return values
    if jaw_mapping_file:
        from arx_env.joint_head_real_motion import feedback_to_command
        measured = raw(plan['ticks'][0]['target'])
        hold = feedback_to_command(measured)
        assert abs(hold[6] - .077815) < 1e-12 and abs(hold[13] - .07633375) < 1e-12
        assert physical(measured)['left_gripper'] == physical(measured)['right_gripper'] == 83.
    clock = [100.]
    with patch('time.monotonic', lambda: clock[0]):
        backend = ref.SimulatedBackend(raw(plan['ticks'][0]['target']))
        backend.q[0] += .019  # Planning-start mismatch must not reject preview/initial command.
        import threading
        sessions=[]
        class RecordingProbe:
            def __init__(self):
                self.ready=threading.Event()  # Recording readiness must not delay motion.
                self.done=threading.Event();self.error=None;self.events=[];self.terminal=None
            def submit(self,kind,**fields):self.events.append((kind,fields))
            def finish(self,status,reason=None):
                if self.terminal is None:self.terminal=(status,reason)
                self.done.set()
            def snapshot(self):return dict(done=self.done.is_set(),error=self.error)
        def recording_factory(payload,on_fault):
            session=RecordingProbe();session.timeline=payload['execution_timeline'];session.on_fault=on_fault
            sessions.append(session);return session
        owner = Owner(backend, SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State),recording_factory=recording_factory)
        for _ in range(42):
            owner.tick(); clock[0] += .01
        assert owner.snapshot()['stationary']
        before = list(backend.events)
        if jaw_mapping_file:
            wrong = deepcopy(source)
            wrong['calibration_profile']['gripper']['mapping'] = 'legacy-pad-correction-2mm'
            wrong_plan = deepcopy(plan)
            wrong_plan['calibration_profile_digest'] = _digest(wrong['calibration_profile'])
            try: owner.submit(dict(action='preview_frozen',plan=wrong_plan,source=wrong))
            except ValueError as error: assert 'gripper mapping' in str(error), error
            else: raise AssertionError('wrong jaw mapping accepted')
            assert backend.events == before
            wrong = deepcopy(source)
            wrong['calibration_profile']['model_binding']['jaw_mapping_sha256'] = 'wrong'
            wrong_plan['calibration_profile_digest'] = _digest(wrong['calibration_profile'])
            try: owner.submit(dict(action='preview_frozen',plan=wrong_plan,source=wrong))
            except ValueError as error: assert 'artifact digest' in str(error), error
            else: raise AssertionError('wrong jaw artifact accepted')
            assert backend.events == before
        if jaw_mapping_file:
            import json
            import tempfile
            with tempfile.TemporaryDirectory() as directory:
                binding_path=Path(directory)/'binding.json'
                binding_path.write_text(json.dumps(binding))
                owner.hardware=True; owner.hardware_binding=binding_path
                try:
                    try:owner.submit(dict(action='preview_frozen',plan=plan,source=real))
                    except ValueError as error:assert '证据未通过' in str(error),error
                    else:raise AssertionError('old rollout-only plan accepted by hardware preview')
                    geometric=deepcopy(plan)
                    geometric['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
                        timeline_digest=geometric['timeline_digest'],source_digest=_digest(real),
                        collision_coverage=['base_link','left','right'],max_sample_step_rad=.01)
                    # Keep old simulation success to ensure it cannot bypass receipt checks.
                    for key,value in [('timeline_digest','wrong'),('source_digest','wrong'),
                                      ('collision_coverage',['left','right']),('max_sample_step_rad',.02)]:
                        bad=deepcopy(geometric);bad['planning_validation'][key]=value
                        try:owner.submit(dict(action='preview_frozen',plan=bad,source=real))
                        except ValueError:pass
                        else:raise AssertionError('invalid hardware geometry receipt accepted: '+key)
                    good=owner.submit(dict(action='preview_frozen',plan=geometric,source=real))
                    assert good['token'] and backend.events==before, 'hardware preview moved fake backend'
                finally:
                    owner.hardware=False;owner.hardware_binding=None
        # Per-tick conversion must not hold owner.lock: a fresh fake SDK sample
        # can be published after >100 ms of validation, without relaxing freshness.
        import real_control
        entered,resume=threading.Event(),threading.Event()
        original_validate=real_control.validate_physical14
        def blocked_validate(value,*args,**kwargs):
            if not entered.is_set():
                entered.set()
                if not resume.wait(2):raise AssertionError('test conversion was not released')
            return original_validate(value,*args,**kwargs)
        with patch.object(real_control,'validate_physical14',side_effect=blocked_validate), ThreadPoolExecutor(max_workers=2) as pool:
            preview_task=pool.submit(owner.submit,dict(action='preview_frozen',plan=plan,source=source))
            try:
                assert entered.wait(1)
                for _ in range(20):
                    clock[0]+=.01
                    assert pool.submit(owner.tick).result(timeout=.5) is True
            finally:resume.set()
            assert preview_task.result(timeout=1)['token']
        assert backend.events==before, 'concurrent preview/cache refresh moved fake hardware'
        preview = owner.submit(dict(action='preview_frozen',plan=plan,source=source))
        assert backend.events == before, 'preview must not send'
        assert preview['control_hz']==30 and preview['initial_target']==plan['ticks'][0]['target']
        backend.q[0] += .02  # A post-preview pose change returns smoothly; it does not reject.
        confirm = dict(action='confirm',token=preview['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True)
        owner.submit(confirm)
        # Starting on the owner loop must consume the previewed executor, never
        # revalidate an entire timeline while its feedback sample grows stale.
        with patch.object(real_control,'FrozenExecutor',side_effect=AssertionError('owner loop repeated full-plan validation')):
            owner.tick(); clock[0] += .01
        host_pause=False
        for _ in range(1000):
            # Model measured feedback using the inverse feedback mapping, not command units.
            if owner.runner.runtime_started:
                assert owner.runner.runtime_ready.wait(2), 'CPU initialization compilation stalled'
            if owner.runner and owner.runner.last_sent_index >= 0:
                backend.q = raw(owner.runner.frozen.ticks[owner.runner.last_sent_index]['target'])
                if not host_pause:
                    clock[0]+=.15;host_pause=True  # Fresh SDK feedback after a host pause may continue.
            owner.tick(); clock[0] += .01
            if owner.runner.status=='READY':break
        assert owner.runner.status == 'READY', owner.snapshot()
        assert host_pause and owner.runner.frozen.schedule_slips and owner.runner.maximum_control_gap_s>=.15
        submitted_before_seal=sum(kind=='command'for kind,row in sessions[0].events)
        for _ in range(10):
            if sessions[0].terminal is not None:break
            owner.tick();clock[0]+=.01
        assert sessions[0].terminal == ('completed',None)
        assert sum(kind=='command'for kind,row in sessions[0].events)==submitted_before_seal
        commands=[row for kind,row in sessions[0].events if kind=='command']
        runtime=sessions[0].timeline
        assert runtime['initialization_duration_s']>0
        assert abs(runtime['initialization']['actual_start']['left_arm_joints'][0]-.039)<1e-12
        assert [row['target'] for row in commands] == [row['target'] for row in runtime['commands']]
        assert [row['tick'] for row in commands] == list(range(len(runtime['commands'])))
        assert runtime['control_hz']==30
        suffix=[row for row in runtime['commands'] if row['source_tick'] is not None]
        assert [row['target'] for row in suffix]==[row['target'] for row in plan['ticks']]
        assert [row['source_tick'] for row in suffix]==list(range(len(plan['ticks'])))
        assert all(abs(row['time_s']-runtime['initialization_duration_s']-parent['time_s'])<1e-9
                   for row,parent in zip(suffix,plan['ticks'])), 'parent target times changed'
        assert owner.runner.frozen.plan == plan, 'runtime compilation mutated the reviewed plan'
        assert any(kind=='feedback' and 'raw' in row for kind,row in sessions[0].events)
        sends = [event for event in backend.events if event[0]=='send']
        assert len(sends) == 2*len(runtime['commands']), sends
        assert [event[2] for event in sends[::2]] == [sdk(t['target']) for t in runtime['commands']]
        assert owner.runner.frozen.snapshot()['targets_submitted']
        state = owner.snapshot()
        assert state['sample_age_s'] < .02 and state['physical_feedback'] == physical(backend.q)
        assert state['jaw_gap_mm'] == [state['physical_feedback']['left_gripper'], state['physical_feedback']['right_gripper']]
        try: owner.submit(confirm)
        except ValueError: pass
        else: raise AssertionError('consumed token accepted')
        for _ in range(42): owner.tick(); clock[0] += .01
        preview = owner.submit(dict(action='preview_frozen',plan=plan,source=source))
        owner.submit({'action':'stop'})
        try: owner.submit(dict(confirm,token=preview['token']))
        except ValueError: pass
        else: raise AssertionError('stopped token accepted')
        owner.tick()
        assert owner.control.state is State.STOPPED
        owner.hardware = True
        try: owner.submit(dict(action='preview_frozen',plan=plan,source=source))
        except ValueError as error: assert 'provenance' in str(error)
        else: raise AssertionError('fake provenance reached hardware')
        owner.runner.status = 'READY'
        backend.q[0] += .1
        clock[0] += .01
        owner.tick()
        assert owner.runner.status == 'READY' and owner.control.state is not State.FAULT
        assert owner.runner.target_error['joint_rad']>.05, 'endpoint error was hidden'
        owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
        assert owner.tick() is False and not backend.is_connected
        # Synthetic hardware-shaped backend: no SDK factory, CAN or device imports.
        # Raw out-of-range feedback is retained; every issued hold/command is legal.
        class Gain:
            def __init__(self,count):self.p=[0.]*count;self.d=[0.]*count
            def kp(self):return self.p
            def kd(self):return self.d
        outside = ref.SimulatedBackend(raw(plan['ticks'][0]['target']))
        outside.q[0]=.054  # 4 mrad boundary noise; larger excess is rejected separately.
        outside.q[6]=interpolate(87.,GAP_MM['left'],FEEDBACK['left'])
        limits=outside.get_bimanual_joint_position_limits();limits[0]=[-.05,.05]
        outside.get_bimanual_joint_position_limits=lambda:limits
        outside._require_connected=lambda:None
        outside._robot_configs={side:SimpleNamespace(robot_model='X5') for side in ('left','right')}
        outside._controller_configs={side:SimpleNamespace(default_kp=[80,70,70,30,30,20],
            default_kd=[2,2,2,1,1,.7],default_gripper_kp=5.,default_gripper_kd=.2) for side in ('left','right')}
        outside.config=SimpleNamespace(gravity_compensation=True)
        outside._arx5=SimpleNamespace(Gain=Gain);outside.manual_jaws={'left','right'}
        gains=[]
        outside._controllers={side:SimpleNamespace(set_gain=lambda gain,side=side:gains.append((side,gain)))
                              for side in ('left','right')}
        original_send=outside.send;sent=[]
        def legal_send(side,target):
            assert all(lo<=value<=hi for value,(lo,hi) in zip(target,limits)),target
            sent.append(list(target));original_send(side,target)
        outside.send=legal_send
        class BrokenRecording(RecordingProbe):
            def __init__(self):
                super().__init__();self.error='camera stalled';self.done.set()
            def submit(self,*args,**kwargs):raise OSError('record writer failed')
            def finish(self,*args,**kwargs):raise OSError('record seal failed')
        failed_sessions=[]
        def broken_factory(payload,on_fault):
            session=BrokenRecording();session.timeline=payload['execution_timeline'];failed_sessions.append(session)
            on_fault(session.error);return session
        outside_owner=Owner(outside,SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State),recording_factory=broken_factory)
        for _ in range(42):outside_owner.tick();clock[0]+=.01
        outside_preview=outside_owner.submit(dict(action='preview_frozen',plan=plan,source=source))
        # Select only the adapter's hardware gain branch using the in-memory stub.
        outside_owner.hardware=True
        outside_owner.submit(dict(confirm,token=outside_preview['token']))
        outside_owner.tick();clock[0]+=.01
        for _ in range(2500):
            runner=outside_owner.runner
            if runner.runtime_started:assert runner.runtime_ready.wait(2)
            if runner.last_sent_index>=0:
                outside.q=raw(runner.frozen.ticks[runner.last_sent_index]['target'])
                outside.q[1]+=.2  # Ordinary tracking error stays inside this joint's legal range.
            outside_owner.tick();clock[0]+=.01
            if runner.status=='READY':break
            assert runner.status!='FAULT',outside_owner.snapshot()
        assert outside_owner.runner.status=='READY',outside_owner.snapshot()
        initialization=failed_sessions[0].timeline['initialization']
        assert initialization['actual_start']['left_arm_joints'][0]==.054
        assert abs(initialization['actual_start']['left_gripper']-87.)<1e-9
        assert initialization['legal_start']['left_arm_joints'][0]==.05
        assert initialization['legal_start']['left_gripper']==83.
        assert initialization['command_projection'] and sent and len(gains)==2
        assert outside_owner.recording_error=='camera stalled' and outside_owner.control.state is State.READY
        assert outside_owner.runner.frozen.plan==plan
        assert outside_owner.runner.frozen.tracking_exceeded_samples>0, 'ordinary measured error was not observed'
        # Finite/fresh feedback and real SDK/gain identity failures still protect.
        outside.q[0]=float('nan');clock[0]+=.01
        outside_owner.tick()
        assert outside_owner.control.state is State.FAULT
        stale=ref.SimulatedBackend(raw(plan['ticks'][0]['target']))
        original_read=stale.read_calibration_state
        stale.read_calibration_state=lambda:dict(original_read(),sample_time_s=clock[0]-.2)
        stale_owner=Owner(stale,SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State))
        stale_owner.tick()
        assert stale_owner.control.state is State.FAULT and 'stale' in stale_owner.control.reason
        rejected=ref.SimulatedBackend(raw(plan['ticks'][0]['target']))
        def sdk_reject(*args):raise RuntimeError('SDK gain rejected')
        rejected.begin=sdk_reject
        rejected_owner=Owner(rejected,SimpleNamespace(PedalTakeover=Takeover,TakeoverState=State))
        for _ in range(42):rejected_owner.tick();clock[0]+=.01
        rejected_preview=rejected_owner.submit(dict(action='preview_frozen',plan=plan,source=source))
        rejected_owner.submit(dict(confirm,token=rejected_preview['token']))
        for _ in range(60):
            rejected_owner.tick();clock[0]+=.01
            if rejected_owner.runner.runtime_started:assert rejected_owner.runner.runtime_ready.wait(2)
            if rejected_owner.runner.status=='FAULT':break
        assert rejected_owner.control.state is State.FAULT and 'SDK gain rejected' in rejected_owner.runner.reason
    print('real control: unchanged 30 Hz commands, smooth return, raw mapping, observation-only endpoint and provenance gates passed')


if __name__ == '__main__':
    main(Path(sys.argv[1]))
    import json
    import tempfile
    import jaw_mapping_v2 as candidate
    from arx_env.joint_head_jaw_mapping import configure_mapping
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'jaw.json'
        rows = {side: dict(candidate_use_allowed=True,
                          command_sdk_knots=list(candidate.COMMAND[side]),
                          feedback_sdk_knots=list(candidate.FEEDBACK[side]),
                          metal_knots_mm=list(candidate.GAP_MM[side])) for side in ('left','right')}
        path.write_text(json.dumps(dict(schema_version='jaw_mapping_audit.v1', reinterpreted_mapping=rows)))
        try: main(Path(sys.argv[1]), path)
        finally: configure_mapping(None)
