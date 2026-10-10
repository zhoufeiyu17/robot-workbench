"""Wall-clock L1 fault receipts. CPU protocol fake only; not ARX/PhysX qualification.

Run with --evidence-dir PATH to preserve the predeclared matrix, each outcome,
host metadata and all normal-returning SDK stop samples. No GPU or hardware.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import socket
import sys
from tempfile import TemporaryDirectory
import threading
import time
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request,build_opener,ProxyHandler

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),
               str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from test_owner_clock import ClockBackend,Safety,calibration,jaw
from test_collection_physx_protocol import protocol_fixture
from robot_workbench.runtime import Runtime
from robot_workbench.service import Application,Device,serve
from real_recording import RecordingSession,RealRecorder,CAMERAS


MATRIX={
    'permission_epoch':['nonholder','wrong_environment','revoked_review'],
    'late_callback':['calibration_stop','calibration_install','collection_handoff'],
    'restart':['calibration_review','collection_review','exclusive_writer'],
    'recording_disk':['queue_overflow','destination_unwritable','command_log_unwritable'],
    'camera':['directory_absent','wrong_serial','pair_mismatch'],
    'sdk':['blocked_send','feedback_frozen','feedback_error'],
    'handoff_race':['queued_confirm','running_motion','writer_not_exited'],
    'installation':['pending_review','queued_confirm','running_snapshot'],
    'ui_disconnect':['preview_response_lost','running_disconnect','stop_response_lost'],
}


def require(condition,message):
    if not condition:raise AssertionError(message)


def rejected(function,contains=None):
    try:function()
    except (ValueError,PermissionError,RuntimeError,HTTPError,BlockingIOError) as error:
        if contains is not None:require(contains in str(error),str(error))
        return str(error)
    raise AssertionError('forbidden action succeeded')


class SDK(ClockBackend):
    """Scripted L1 feedback; all execution and recording business code is real."""
    def __init__(self,q,clock_id):
        super().__init__(time.monotonic,q,source_clock_id=clock_id)
        self.send_entered=threading.Event();self.send_release=None
        self.send_delay_s=0.;self.read_failure=False;self.trace=[];self.runtime=None
    def read_calibration_state(self):
        if self.read_failure:raise OSError('L1 injected SDK feedback transport failure')
        return super().read_calibration_state()
    def send(self,side,q):
        row=dict(side=side,started=time.monotonic(),epoch=self.runtime.owner.epoch,target=list(q))
        self.trace.append(row);self.send_entered.set()
        if self.send_release is not None:
            if not self.send_release.wait(3):raise TimeoutError('bounded L1 SDK injection expired')
        if self.send_delay_s:time.sleep(self.send_delay_s)
        super().send(side,q)
        row['returned']=time.monotonic()


class Harness:
    def __init__(self):
        self.temp=TemporaryDirectory();self.root=Path(self.temp.name)
        _,self.source,self.plan,self.binding,self.mapping=protocol_fixture(self.root)
        jaw.configure_mapping(self.mapping)
        target=self.plan['ticks'][0]['target']
        self.q=[*target['left_arm_joints'],jaw.feedback_sdk('left',target['left_gripper']),
                *target['right_arm_joints'],jaw.feedback_sdk('right',target['right_gripper'])]
        self.recording_factory=None;self.sessions=[];self.release_gates=[];self.owner_events=[]
        self.runtime=None;self.sdk=None;self.counter=0
        self.allow_tick=threading.Event();self.allow_tick.set();self.tick_paused=threading.Event()
        def factory():
            sdk=SDK(self.q,self.source['clock']['source_clock_id'])
            runtime=Runtime(calibration,Safety,lambda:sdk,device_id='l1-wall-device',environment='fake',
                lock_root=self.root/'locks',jaw_mapping_file=self.mapping,hardware_binding=self.root/'binding.json',
                recording_factory=self.recording_factory,
                initialize_backend=lambda backend:backend.set_damping_hold(manual=True),
                event=lambda kind,**fields:self.owner_events.append(dict(kind=kind,**fields)))
            sdk.runtime=runtime;self.sdk=sdk;self.runtime=runtime
            return runtime
        def drive(runtime,closed):
            while not closed.is_set():
                if not self.allow_tick.is_set():
                    self.tick_paused.set();self.allow_tick.wait(.005);continue
                self.tick_paused.clear();started=time.monotonic()
                if not runtime.tick():break
                closed.wait(max(0.,.01-(time.monotonic()-started)))
        self.device=Device('l1-wall-device','fake',factory,drive=drive)
        self.app=Application({'fake':self.device,'physical':Device('unconfigured-physical','physical',None)})
        self.server=serve(self.app,port=0)
        self.thread=threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.01),daemon=True)
        self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        self.http=build_opener(ProxyHandler({}))
        self.action('connect');self.wait(lambda:self.runtime and self.runtime.owner.stationary)
        require(self.runtime.owner.control_clock is None,'wall-clock qualification cannot inject a simulation control clock')
    def state(self,business='calibration'):
        with self.http.open(self.url+'/api/state?environment=fake&business='+business,timeout=2) as response:
            return json.loads(response.read())
    def body(self,business='calibration',**fields):
        self.counter+=1
        return dict(environment='fake',business=business,request_id=str(self.counter),**fields)
    def action(self,action,business='calibration',**fields):
        body=self.body(business,**fields)
        req=Request(self.url+'/api/actions/'+action,json.dumps(body).encode(),headers={
            'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
        with self.http.open(req,timeout=2) as response:return json.loads(response.read())
    def drop_response(self,action,**fields):
        body=json.dumps(self.body(**fields)).encode()
        headers=(f'POST /api/actions/{action} HTTP/1.1\r\nHost: 127.0.0.1:{self.server.server_port}\r\n'
                 f'Origin: {self.url}\r\nX-CSRF-Token: {self.app.csrf}\r\nContent-Type: application/json\r\n'
                 f'Content-Length: {len(body)}\r\nConnection: close\r\n\r\n').encode()
        with socket.create_connection(('127.0.0.1',self.server.server_port),timeout=2) as sock:
            sock.sendall(headers+body);sock.shutdown(socket.SHUT_WR)
    def wait(self,predicate,timeout=4):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=predicate()
            if value:return value
            time.sleep(.005)
        raise AssertionError('wall wait timed out; '+str(self.runtime.snapshot(self.credentials[self.runtime.holder]) if self.runtime else 'no runtime'))
    @property
    def credentials(self):return self.device.credentials
    def preview(self,business='calibration'):
        if business=='calibration':
            q=self.runtime.owner.cache['calibration_feedback']['qpos'][:];q[0]+=.03
            command=dict(action='preview',active_arm='left',q_target=q)
        else:command=dict(action='preview_frozen',source=self.source,plan=self.plan)
        return self.runtime.submit(self.credentials[business],command)
    def confirm(self,review,business='calibration'):
        return self.runtime.submit(self.credentials[business],dict(action='confirm',token=review['token'],
            confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
    def collect(self):
        self.runtime.handoff(self.credentials['calibration'],'collection')
        self.confirm(self.preview('collection'),'collection')
    def completed(self):
        self.wait(lambda:self.runtime.owner.runner and self.runtime.owner.runner.status in ('READY','FAULT','STOPPED'))
        return self.runtime.owner.runner
    def assert_collection_complete(self):
        runner=self.completed()
        require(runner.status=='READY',runner.reason)
        require(runner.frozen.status=='completed','data failure altered D motion outcome')
        commands=[row for row in self.owner_events if row['kind']=='command']
        require([row['tick'] for row in commands]==list(range(len(runner.runtime['commands']))),
                'D commands skipped or burst reconstruction changed order')
        require(not self.runtime.owner.stop_requested,'data failure requested motion stop')
        return dict(motion_status=runner.status,command_count=len(commands),recording_error=self.runtime.owner.recording_error)
    def stop(self):return self.action('stop',business=self.runtime.holder)
    def recording(self,kind):
        def factory(payload,on_fault):
            path=self.root/'recording'
            kwargs={}
            if kind=='destination_unwritable':path.write_text('occupied by a file')
            if kind=='directory_absent':kwargs['camera_directories']=dict.fromkeys(CAMERAS,str(self.root/'absent'))
            def recorder(*args,**options):
                if kind=='queue_overflow':
                    entered.set();require(release.wait(3),'bounded disk injection expired')
                result=RealRecorder(*args,**options)
                if kind=='command_log_unwritable':(result.root/'commands.jsonl').mkdir()
                return result
            entered,release=threading.Event(),threading.Event();self.release_gates.append(release)
            session=RecordingSession(path,payload['source'],payload['plan'],self.binding,
                execution_timeline=payload['execution_timeline'],environment='fake',execution_owner=payload['execution_owner'],
                resident_deadline_wall_ns=time.time_ns()+60_000_000_000,
                on_fault=on_fault,capacity=1 if kind=='queue_overflow' else 512,recorder_factory=recorder,**kwargs)
            self.sessions.append(session)
            if kind in ('wrong_serial','pair_mismatch'):
                from test_real_observation import fixture
                def metadata(meta):
                    raw=self.runtime.owner.current_feedback
                    ns=raw['source_time_ns']
                    meta.update(environment='fake',simulation_only=True,evidence_source='fake_device_rgbd',
                        clock_domain='simulation',source_clock_id=self.source['clock']['source_clock_id'],
                        source_time_ns=ns,physics_step=raw['physics_step'],timestamp_kind='simulation_step',
                        timestamp_domain='simulation',device_timestamp_ms=ns/1e6,depth_device_timestamp_ms=ns/1e6,
                        host_receive_ns=time.time_ns(),owner_available=True,owner_pid=os.getpid(),
                        command_epoch=self.runtime.owner.epoch,runtime_nonce=self.runtime.nonce)
                    if kind=='wrong_serial':meta['serial']='unexpected-camera'
                    else:meta['depth_device_timestamp_ms']+=100.
                session.submit('rgbd',camera=CAMERAS[0],blob=fixture(1,metadata))
            return session
        self.runtime.owner.recording_factory=factory
    def close(self):
        self.allow_tick.set()
        for gate in self.release_gates:gate.set()
        if self.sdk:
            self.sdk.read_failure=False;self.sdk.frozen_sample=None
            if self.sdk.send_release:self.sdk.send_release.set()
        for session in self.sessions:
            session.finish('stopped','L1 fixture cleanup');session.done.wait(2);session.thread.join(1)
        if self.runtime and self.runtime.fd is not None:
            if self.runtime.owner.backend.is_connected:
                self.runtime.submit(self.credentials[self.runtime.holder],dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                if self.device.thread:self.device.thread.join(2)
            self.runtime.close()
        self.device.closed.set();self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.temp.cleanup()


def execute(category,variant):
    h=Harness()
    try:
        r=h.runtime;c=h.credentials
        if category=='permission_epoch':
            before=len(h.sdk.trace)
            if variant=='nonholder':reason=rejected(lambda:h.preview('collection'),'交接')
            elif variant=='wrong_environment':
                credential=dict(c['calibration'],environment='physical')
                reason=rejected(lambda:r.submit(credential,dict(action='stop')),'another runtime/device/environment')
            else:
                review=h.preview();h.stop();reason=rejected(lambda:h.confirm(review))
            time.sleep(.03);require(len(h.sdk.trace)==before,'unauthorized request sent a target')
            return dict(rejection=reason,unauthorized_targets=0)
        if category=='late_callback':
            entered,release=threading.Event(),threading.Event();h.release_gates.append(release);result=[]
            business='collection' if variant=='collection_handoff' else 'calibration'
            if business=='collection':r.handoff(c['calibration'],'collection')
            obj,name=(r.owner,'source_validator') if business=='collection' else (calibration,'make_plan')
            original=getattr(obj,name)
            def delayed(*args,**kwargs):
                value=original(*args,**kwargs);entered.set();require(release.wait(3),'compiler delay expired');return value
            def preview():
                try:result.append(h.preview(business))
                except Exception as error:result.append(error)
            with patch.object(obj,name,delayed):
                worker=threading.Thread(target=preview);worker.start();require(entered.wait(2),'actual compiler did not reach barrier')
                if variant=='calibration_stop':h.stop()
                elif variant=='calibration_install':r.notify_calibration_installed('L1-installed-v2')
                else:r.handoff(c['collection'],'calibration')
                release.set();worker.join(2)
            require(not worker.is_alive() and isinstance(result[0],ValueError),str(result))
            require(not r.owner.pending and not h.sdk.trace,'late callback revived a token or motion')
            return dict(rejection=str(result[0]),late_tokens=0,sdk_targets=0)
        if category=='restart':
            business='collection' if variant=='collection_review' else 'calibration'
            if business=='collection':r.handoff(c['calibration'],'collection')
            review=h.preview(business)
            if variant=='exclusive_writer':
                constructed=[]
                def forbidden():constructed.append(True);return h.sdk
                reason=rejected(lambda:Runtime(calibration,Safety,forbidden,device_id=r.device_id,
                    environment='fake',lock_root=h.root/'locks'))
                require(not constructed,'second runtime constructed an SDK before exclusive lock')
                return dict(rejection=reason,second_sdk_constructions=0)
            old_nonce=r.nonce;old_credential=deepcopy(c[business])
            r.submit(c[business],dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'));h.device.thread.join(2);r.close()
            h.device.runtime=None;h.device.credentials={};h.action('connect');h.wait(lambda:h.runtime is not r and h.runtime.owner.stationary)
            require(h.runtime.nonce!=old_nonce,'restart reused runtime nonce')
            rejected(lambda:h.runtime.submit(old_credential,dict(action='stop')))
            if business=='collection':h.runtime.handoff(h.credentials['calibration'],'collection')
            reason=rejected(lambda:h.confirm(review,business))
            require(not h.sdk.trace,'restart resumed an old reviewed plan')
            return dict(rejection=reason,old_runtime_revoked=True,sdk_targets=0)
        if category in ('recording_disk','camera'):
            h.recording(variant);h.collect();result=h.assert_collection_complete()
            for gate in h.release_gates:gate.set()
            session=h.sessions[0];require(session.done.wait(3),'data worker did not terminate');session.thread.join(1)
            require(session.error is not None,'fault injection did not produce a recording error')
            expected={'queue_overflow':'queue overflow','destination_unwritable':'File exists',
                'command_log_unwritable':'Is a directory','directory_absent':'No such file',
                'wrong_serial':'camera identity','pair_mismatch':'timestamp'}[variant]
            require(expected.lower() in session.error.lower(),f'wrong first fault: {session.error}')
            require(not session.manifest or session.manifest['capture_status']=='incomplete','data fault produced valid raw')
            return dict(result,first_recording_error=session.error,capture_status=(session.manifest or {}).get('capture_status','manifest_absent'),
                        writer_exited=not session.thread.is_alive())
        if category=='sdk':
            if variant=='blocked_send':
                release=threading.Event();h.sdk.send_release=release;h.release_gates.append(release)
                h.confirm(h.preview());require(h.sdk.send_entered.wait(2),'SDK send not reached')
                started=time.monotonic();receipt=h.stop();elapsed=time.monotonic()-started
                require(not receipt['stop']['stop_hold_return_confirmed'],'blocked SDK claimed stopped')
                require(receipt['stop']['in_flight_calls'],'in-flight call missing')
                reason=rejected(lambda:r.handoff(c['calibration'],'collection'),'in flight')
                barrier=receipt['stop']['accepted_monotonic_s'];release.set()
                h.wait(lambda:r.snapshot(c['calibration'])['stop_receipt']['stop_hold_return_confirmed'])
                time.sleep(.03)
                require(not [row for row in h.sdk.trace if row['started']>barrier],'old target started after stop barrier')
                return dict(stop_ack_wall_s=elapsed,while_blocked='unconfirmed',rejection=reason,old_targets_after_barrier=0)
            h.confirm(h.preview());require(h.sdk.send_entered.wait(2),'SDK send not reached')
            if variant=='feedback_frozen':h.sdk.frozen_sample=time.monotonic()
            else:h.sdk.read_failure=True
            h.wait(lambda:r.owner.runner.status=='FAULT' or not r.owner.feedback_healthy)
            count=len(h.sdk.trace);time.sleep(.15)
            require(len(h.sdk.trace)==count,'feedback failure kept sending targets')
            state=r.snapshot(c['calibration']);h.sdk.read_failure=False;h.sdk.frozen_sample=None
            time.sleep(.1);require(len(h.sdk.trace)==count,'restored feedback resumed old motion')
            return dict(state=state['state'],reason=state.get('reason'),targets_after_failure=0)
        if category=='handoff_race':
            if variant=='writer_not_exited':
                h.recording('queue_overflow');h.collect();h.assert_collection_complete()
                h.wait(lambda:r.owner.stationary)
                require(h.sessions and h.sessions[0].thread.is_alive(),'writer injection already ended')
                reason=rejected(lambda:r.handoff(c['collection'],'calibration'),'writer')
                for gate in h.release_gates:gate.set()
                h.wait(lambda:h.sessions[0].done.is_set() and not h.sessions[0].thread.is_alive())
                h.wait(lambda:r.owner.stationary)
                r.handoff(c['collection'],'calibration')
                return dict(rejection=reason,writer_finished_before_handoff=True)
            review=h.preview()
            if variant=='queued_confirm':
                h.allow_tick.clear();require(h.tick_paused.wait(1),'owner scheduler did not pause')
                try:
                    h.confirm(review);reason=rejected(lambda:r.handoff(c['calibration'],'collection'))
                finally:h.allow_tick.set()
            else:
                h.confirm(review);require(h.sdk.send_entered.wait(2),'motion did not begin')
                reason=rejected(lambda:r.handoff(c['calibration'],'collection'),'active')
            require(r.holder=='calibration','handoff occurred during motion')
            h.stop();return dict(rejection=reason,holder_unchanged=True)
        if category=='installation':
            r.notify_calibration_installed('L1-v1');review=h.preview();before=len(h.sdk.trace)
            if variant=='running_snapshot':
                h.confirm(review);require(h.sdk.send_entered.wait(2),'motion did not begin')
                accepted=deepcopy(r.execution_calibration);epoch=r.owner.epoch
                r.notify_calibration_installed('L1-v2');runner=h.completed()
                require(runner.status=='READY',runner.reason)
                require(r.execution_calibration==accepted and accepted['snapshot_id']=='L1-v1','install changed frozen running snapshot')
                require(r.owner.epoch==epoch,'install revoked an already running plan')
                return dict(running_snapshot_preserved=True,active_snapshot=r.calibration_snapshot_id)
            if variant=='queued_confirm':
                h.allow_tick.clear();require(h.tick_paused.wait(1),'owner scheduler did not pause')
                try:h.confirm(review);r.notify_calibration_installed('L1-v2')
                finally:h.allow_tick.set()
                time.sleep(.06);require(r.owner.runner is None,'queued old review started after install')
            else:
                r.notify_calibration_installed('L1-v2');rejected(lambda:h.confirm(review))
            require(len(h.sdk.trace)==before,'installed revision fence sent old targets')
            return dict(old_tokens_revoked=True,old_targets=0)
        if category=='ui_disconnect':
            q=h.q[:6]+h.q[7:13];q[0]+=.03
            if variant=='preview_response_lost':
                h.drop_response('manual_preview',q12=q,jaw_gap_mm=[83.,83.])
                h.wait(lambda:h.app.reviews.get(('fake','calibration')))
                review=h.state()['workflow']['review'];h.stop()
                rejected(lambda:h.action('manual_execute',review_id=review['id'],physical_clearance=True))
                require(not h.sdk.trace,'lost preview response automatically executed')
            elif variant=='running_disconnect':
                h.confirm(h.preview());require(h.sdk.send_entered.wait(2),'motion did not begin')
                before=len(h.sdk.trace)
                with socket.create_connection(('127.0.0.1',h.server.server_port),timeout=2):pass
                time.sleep(.05);require(len(h.sdk.trace)>before,'UI disconnect silently paused motion')
                h.stop();count=len(h.sdk.trace);time.sleep(.04)
                h.state();require(len(h.sdk.trace)==count,'UI reconnect resumed motion')
            else:
                review=h.preview();h.confirm(review);require(h.sdk.send_entered.wait(2),'motion did not begin')
                h.drop_response('stop')
                h.wait(lambda:r.stop_receipt and r.stop_receipt['stop_hold_return_confirmed'])
                rejected(lambda:h.confirm(review));count=len(h.sdk.trace);h.state();time.sleep(.03)
                require(len(h.sdk.trace)==count,'lost stop reply/reconnect resumed old motion')
            return dict(late_or_reconnected_motion_targets=0,review_revoked=True)
        raise AssertionError('unknown matrix case')
    finally:h.close()


def stop_samples(count=30):
    h=Harness();rows=[]
    try:
        for index in range(count):
            h.wait(lambda:h.runtime.owner.stationary)
            h.sdk.send_delay_s=(0.,.005,.015)[index%3]
            h.sdk.send_entered.clear();h.confirm(h.preview())
            require(h.sdk.send_entered.wait(2),'normal SDK target not reached')
            received=time.monotonic();reply=h.stop();response=time.monotonic()
            receipt=reply['stop'];barrier=receipt['accepted_monotonic_s']
            h.wait(lambda:h.runtime.snapshot(h.credentials['calibration'])['stop_receipt']['stop_hold_return_confirmed'])
            confirmed=time.monotonic()
            require(not [row for row in h.sdk.trace if row['started']>barrier],'target started after software barrier')
            rows.append(dict(index=index,normal_sdk_delay_s=h.sdk.send_delay_s,request_started_monotonic_s=received,
                software_barrier_monotonic_s=barrier,request_to_barrier_s=barrier-received,
                http_response_s=response-received,hold_confirmation_observed_s=confirmed-received,
                in_flight_calls=receipt['in_flight_calls'],old_targets_after_barrier=0))
        values=sorted(row['request_to_barrier_s'] for row in rows)
        p95=values[math.ceil(.95*len(values))-1];maximum=max(values)
        return dict(samples=rows,count=len(rows),p95_s=p95,max_s=maximum,passed=p95<=.1 and maximum<=.2,
            measurement='local HTTP client request start to post-revocation software barrier receipt; inclusive local HTTP overhead',
            sdk_profile='predeclared normal-returning scripted SDK, delays 0/5/15ms; blocked cases separately reported',
            physical_stop='unverified',firmware_ack='unsupported',clock='time.monotonic wall domain')
    finally:h.close()


def command_clock_regression():
    h=Harness()
    try:
        h.recording('healthy');h.collect();result=h.assert_collection_complete()
        session=h.sessions[0];require(session.done.wait(3),'normal recorder did not seal')
        require(session.error is None,str(session.error))
        commands=session.recorder.commands
        require(len(commands)==len(h.runtime.owner.runner.runtime['commands']),'normal command log incomplete')
        for row in commands:
            require(abs(round(row['send_monotonic_s']*1e9)-row['send_source_time_ns'])<=1,'command clock changed')
            require(row['physics_step'] is None,'submission masquerades as a completed physics step')
            require(row['feedback_at_send']['source_time_ns']<=row['send_source_time_ns'],'future feedback at submission')
        require(session.manifest['capture_status']=='incomplete','no-camera fixture claimed complete raw')
        return dict(result,passed=True,recorded_commands=len(commands),
            send_minus_feedback_ns=[row['send_source_time_ns']-row['feedback_at_send']['source_time_ns'] for row in commands],
            command_physics_steps=[row['physics_step'] for row in commands],
            capture_status=session.manifest['capture_status'],raw_complete_claimed=False)
    finally:h.close()


def source_manifest():
    return {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for folder in ('robot_workbench','rule_based_workbench','calibration','tests')
            for path in sorted((ROOT/folder).rglob('*.py')) if '__pycache__' not in path.parts}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir',type=Path,required=True)
    parser.add_argument('--case',help='optional category:variant for a bounded debugging run')
    parser.add_argument('--skip-stop-performance',action='store_true')
    args=parser.parse_args();args.evidence_dir.mkdir(parents=True,exist_ok=True)
    path=args.evidence_dir/'l1-fault-matrix.json'
    require(not path.exists(),'preserve prior receipts; choose a new evidence directory')
    cases=[(category,variant) for category,variants in MATRIX.items() for variant in variants]
    if args.case:cases=[tuple(args.case.split(':',1))]
    receipt=dict(schema_version='l1_fault_matrix.v1',started_wall_ns=time.time_ns(),
        qualification_scope='this host only; CPU wall-clock protocol fake; not ARX, PhysX, dynamics or real hardware',
        host=dict(hostname=socket.gethostname(),platform=platform.platform(),machine=platform.machine(),
            python=sys.version,cpu_count=os.cpu_count(),affinity=list(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None,
            cpu_quota='not assigned by this run',load_before=list(os.getloadavg())),
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),source_sha256=source_manifest(),
        existing_coverage=['tests/test_unified_runtime.py:100 fixed CPU schedules retained, not counted as L1 wall evidence',
            'tests/test_http_service.py:real loopback API baseline','rule_based_workbench/test_real_recording.py:recorder/spool baseline'],
        predeclared_cases=[dict(category=c,variant=v) for c,v in cases],results=[],stop_performance=None,command_clock_regression=None)
    def save():
        temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n');temporary.replace(path)
    save()
    for category,variant in cases:
        start=time.monotonic();row=dict(category=category,variant=variant,started_wall_ns=time.time_ns())
        try:row.update(status='passed',evidence=execute(category,variant))
        except Exception as error:row.update(status='failed',error=f'{type(error).__name__}: {error}')
        row['wall_duration_s']=time.monotonic()-start;receipt['results'].append(row);save()
        print(category+':'+variant+' '+row['status'],flush=True)
    if not args.skip_stop_performance:
        try:receipt['stop_performance']=stop_samples()
        except Exception as error:receipt['stop_performance']=dict(passed=False,error=f'{type(error).__name__}: {error}')
        try:receipt['command_clock_regression']=command_clock_regression()
        except Exception as error:receipt['command_clock_regression']=dict(passed=False,error=f'{type(error).__name__}: {error}')
    unchanged=source_manifest()==receipt['source_sha256']
    receipt.update(source_unchanged=unchanged,finished_wall_ns=time.time_ns(),load_after=list(os.getloadavg()),
        passed=unchanged and all(row['status']=='passed' for row in receipt['results']) and
        (args.skip_stop_performance or receipt['stop_performance'].get('passed') is True and receipt['command_clock_regression'].get('passed') is True))
    save();print(json.dumps(dict(passed=receipt['passed'],evidence=str(path))))
    return 0 if receipt['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
