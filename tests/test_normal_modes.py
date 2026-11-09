"""Real Runtime/Owner command queue and normal modes; ideal CPU SDK sink only."""
from copy import deepcopy
from types import SimpleNamespace as NS
import time
import os
import threading
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import test_shared_target_routes as shared
from robot_workbench.inference_owner import InferenceRunner,device_state
from robot_workbench.motion_reference import prepare_reference
from robot_workbench.target_interpreter import gains
from robot_workbench.sdk_process import EXECUTION_CONTEXT_SCHEMA


class NormalBackend(shared.WindowBackend):
    native_reference_api='preserve_reference_v1'
    motion_gain_profile='normal_X5'
    execution_context_capabilities={'schema':EXECUTION_CONTEXT_SCHEMA}
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.mode_gains={s:dict(kp=gains('normal_X5',s)[0],kd=gains('normal_X5',s)[1],gripper_kp=5.,gripper_kd=.2)
                         for s in('left','right')}
        self.gain_calls=[]
        self.native_home={side:'idle'for side in ('left','right')}
        self.home_calls=[]
        self.home_pending=None
        self.native_gripper_calibration_id=shared.jaw.mapping_identity().get('native_calibration_id')

    def bind_execution_context(self,value):
        from robot_workbench.execution_settings import validate_execution_context
        context=validate_execution_context(value)
        if context['controller_profile']!=self.target_interpreter_profile or context['jaw_calibration_id']!=self.native_gripper_calibration_id:
            raise ValueError('CPU context configuration differs')
        self.execution_context=context
        return dict(bound=True)

    def native_values(self,field,stamp):
        if self.window is None:
            return (self.applied_command or self.q)[:]if field=='q_sdk14'else[0.]*14
        return [float(np.interp(stamp,self.window['host_times_s'],np.asarray(self.window[field])[:,j]))for j in range(14)]

    def queue_reference(self,packet):
        # Exact archived append_reference semantics: interpolate(now), retain
        # old knots before the new first knot, append new frames. Not final SO proof.
        now=self.clock();prior=self.window;self.packets.append(deepcopy(packet))
        window=deepcopy(packet);keep=[]if prior is None else[i for i,t in enumerate(prior['host_times_s'])if now<t<packet['host_times_s'][0]]
        for field in('q_sdk14','dq_sdk14','torque_bias_sdk14'):
            window[field]=[self.native_values(field,now)]+([]if prior is None else[prior[field][i]for i in keep])+window[field]
        window['host_times_s']=[now]+([]if prior is None else[prior['host_times_s'][i]for i in keep])+window['host_times_s']
        if len(window['host_times_s'])>64:raise ValueError('native queue capacity')
        self.window=window
        return dict(firmware_ack=False,native_reference_api=self.native_reference_api)

    def read_execution_output(self):
        return dict(q_sdk14=self.native_values('q_sdk14',self.clock()),dq_sdk14=self.native_values('dq_sdk14',self.clock()),
            velocity_limits_sdk14=([2.]*6+[.04])*2,
            gains=deepcopy(self.mode_gains),observations={s:dict(sequence=round(self.clock()*1e6),
                sample_monotonic_ns=round(self.clock()*1e9),controller_instance_id='cpu-'+s)for s in('left','right')},firmware_ack=False)

    def begin(self,side,q,*,preserve_jaws=False):
        if not preserve_jaws:return super().begin(side,q)
        if self.window is not None:
            from robot_workbench.sdk_process import ReferenceWindowRejected
            raise ReferenceWindowRejected('strict begin cannot replace an existing reference; use its continuation')
        from arx_env.joint_head_real_motion import feedback_to_command
        output=self.read_execution_output()
        command=list(feedback_to_command(q,self.get_bimanual_joint_position_limits(),environment='fake'))
        command[6],command[13]=output['q_sdk14'][6],output['q_sdk14'][13]
        self.applied_command=command[:]
        self.mode_gains[side]=dict(kp=gains('normal_X5',side)[0],kd=gains('normal_X5',side)[1],gripper_kp=5.,gripper_kd=.2)
        return dict(measured_qpos14=q[:],command_sdk14=command,source_output=output)

    def start_native_home(self,side):
        if self.home_pending is not None:raise ValueError('CPU native Home already active')
        self.applied_command=self.read_execution_output()['q_sdk14']
        self.window=None
        self.home_calls.append(side);self.native_home[side]='running'
        self.home_pending=(side,self.clock(),self.applied_command[:])
        return dict(accepted=True,side=side,started_source_s=self.clock())

    def read_calibration_state(self):
        if self.home_pending is not None:
            side,start,initial=self.home_pending
            begin,end=(0,6)if side=='left'else(7,13)
            duration=max(max(abs(x)for x in initial[begin:end]),.5)
            alpha=min(1.,(self.clock()-start)/duration)
            self.applied_command[begin:end]=[x*(1-alpha)for x in initial[begin:end]]
            self.q[begin:end]=self.applied_command[begin:end]
            if self.clock()>=start+duration+.5:
                self.native_home[side]='completed';self.home_pending=None
        return dict(super().read_calibration_state(),native_home=dict(self.native_home))

    def set_damping_hold(self,**kwargs):
        retained=self.read_execution_output()if kwargs.get('preserve_jaws')else None
        if self.home_pending is not None:
            self.native_home[self.home_pending[0]]='cancelled';self.home_pending=None
        result=super().set_damping_hold(**kwargs)
        from arx_env.joint_head_real_motion import feedback_to_command
        self.applied_command=list(feedback_to_command(self.q,self.get_bimanual_joint_position_limits(),environment='fake'))
        for side in ('left','right'):
            self.mode_gains[side]=dict(kp=(np.asarray(gains('normal_X5',side)[0])*.3).tolist(),
                kd=gains('normal_X5',side)[1],gripper_kp=.12,gripper_kd=.15)
        if retained is not None:
            for side,slot in (('left',6),('right',13)):
                self.applied_command[slot]=retained['q_sdk14'][slot]
                for key in ('gripper_kp','gripper_kd'):self.mode_gains[side][key]=retained['gains'][side][key]
        return result

    def set_execution_gains(self,rows):
        assert all(rows[s]['gripper_kp']==self.mode_gains[s]['gripper_kp']>0 for s in rows)
        self.gain_calls.append(deepcopy(rows));self.mode_gains=deepcopy(rows)
        return dict(returned=True,firmware_ack=False)


class NormalModeTests(unittest.TestCase):
    profile='normal_X5'
    setUp=shared.SharedTargetRoutesTest.setUp
    cleanup=shared.SharedTargetRoutesTest.cleanup
    confirm=shared.SharedTargetRoutesTest.confirm

    def setup_ipc(self):
        from robot_workbench.control_owner import ControlOwnerService,ControlRuntimeClient
        from robot_workbench.service import Device
        device=Device('mode-ipc-cpu','fake',None);device.runtime=self.runtime;device.credentials=self.creds
        with patch('robot_workbench.control_owner._identity',return_value=dict(pid=os.getpid(),boot_id='cpu-fixture',start_ticks='cpu-fixture')):
            service=ControlOwnerService(device,Path(self.temp.name)/'mode-ipc')
        thread=threading.Thread(target=service.serve_forever,daemon=True);thread.start()
        def close():
            self.cleanup();device.runtime=None;service.close();thread.join(2)
        self.addCleanup(close)
        client=ControlRuntimeClient(service.descriptor_path);credential=client.register_business('inference')
        return client,credential

    def setup_modes(self,recording=False,recorder_ids=None,finish_kind=None,finish_timeout_s=60.,initial_mode='MODEL',per_episode=False,reviewed=False,reviewed_duration_s=.5):
        self.compiler=patch('robot_workbench.motion_reference.prepare_reference_process',side_effect=prepare_reference)
        self.compiler.start();self.addCleanup(self.compiler.stop)
        mapping=None;initial=None
        if finish_kind:
            import hashlib,json
            root=Path(self.temp.name)
            def pin(name,value):
                p=root/name;p.write_text(json.dumps(value));return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            source=dict(id='arx6',left_can='can3',right_can='can1',left_model='X5',right_model='X5',
                gripper_width_m=.083,cpu_fixture_only=True)
            sides={s:dict(closed_readout=2.,open_readout=1.,width_m=.083)for s in('left','right')}
            for side in sides:
                source[side+'_gripper_closed_readout']=2.;source[side+'_gripper_open_readout']=1.
            native=pin('cpu-endpoints.json',dict(schema='native_gripper_endpoints.v1',robot_id='arx6',
                source=pin('cpu-machine.json',source),sdk_width_m=.082,sides=sides))
            self.finish_mapping=pin('cpu-mapping.json',dict(schema_version='native_gripper_mapping.v1',robot_id='arx6',native_gripper_calibration=native))
            mapping=self.finish_mapping['path'];initial=[0.]*14;initial[6]=.04;initial[13]=.03
        original_runtime=shared.Runtime
        def cpu_runtime(*args,**kwargs):
            assert kwargs['environment']=='fake'
            if mapping:kwargs['device_id']='arx6'  # Schema identity of synthetic CPU pins; never physical.
            return original_runtime(*args,**kwargs)
        with patch.object(shared,'WindowBackend',NormalBackend),patch.object(shared,'Runtime',side_effect=cpu_runtime):
            shared.SharedTargetRoutesTest.create(self,q=initial,mapping=mapping)
        self.runtime.owner.preserve_begin_jaws=True
        self.runtime.handoff(self.creds['calibration'],'inference')
        self.requests=[]
        def predict(request):
            if getattr(self,'model_error',False):raise RuntimeError('CPU model supply unavailable')
            self.requests.append(deepcopy(request));rows=np.zeros((60,14));rows[:,0]=np.linspace(.0001,.001,60)
            rows[:,[6,13]]=np.asarray(request['observation']['device_state14'])[[6,13]]
            return dict(request_id=request['request_id'],action14=rows)
        def factory(owner,proxy,feedback,config):
            return InferenceRunner(owner,proxy,feedback,config,predict=predict,
                adapter=NS(device_to_model=lambda x:x,decode=lambda x,held:dict(decoded_device14=x.tolist(),final_device14=x.tolist(),projections=[])),
                observe=lambda now,fb:dict(source_time=now,device_state14=device_state(fb['qpos']),frame_ids={}))
        normal=dict(brake_duration_s=.5,velocity_limit=.25,acceleration_limit=1.,gain_transition_s=.05,
            teach_kd_scale=.1,recording_required=recording,recorder_ids=(recorder_ids or ['robot','rgb'])if recording else[],
            recording_clock_id='host_wall_time_ns')
        config=dict(model_id='cpu-fixture',duration_s=40,execute_steps=30,normal_modes=normal,normal_initial_mode=initial_mode)
        if reviewed:
            from robot_workbench.inference_model import MIXED366_ID
            from test_infer_curve_policy import POLICY
            normal.update(brake_duration_s=.2,velocity_limit=1.,acceleration_limit=40.)
            config.update(model_id=MIXED366_ID,task='coffee',duration_s=reviewed_duration_s,execute_steps=15,
                reference_planning=deepcopy(POLICY),target_execution='shared_reference_windows',
                maximum_observation_age_s=.5,reference_supply_timeout_s=.5,
                conversion_identity={'mapping':{'command_jaw_limits_mm':[0.,81.],'device_jaw_unit':'recorder_endpoint_mm'}})
        if per_episode:
            config['recording_context_policy']='per_episode_review';normal['recorder_ids']=[]
        if finish_kind:
            from robot_workbench.execution_settings import defaults
            config.update(normal_finish_kind=finish_kind,normal_finish_policy=dict(jaw_mapping=self.finish_mapping,
                execution_settings=defaults(),max_velocity_rad_s=.1,jaw_speed_mm_s=10.,timeout_s=finish_timeout_s))
        self.runtime.configure_inference(config,factory)
        preview=dict(action='preview_inference',duration_s=config['duration_s'])
        if per_episode:preview['recording_context']=dict(session_id='capture-1',recorder_ids=recorder_ids or ['robot','rgb'],clock_id='host_wall_time_ns')
        review=self.runtime.submit(self.creds['inference'],preview)
        self.confirm('inference',review)
        self.until(lambda:self.runtime.owner.runner is not None and bool(self.runtime.owner.runner.target_interpreter)and
            ((self.runtime.owner.runner.mode=='HOLD'and self.runtime.owner.runner.normal_job is None)if initial_mode=='HOLD'else
             self.runtime.owner.runner.target_interpreter.reference.metadata.get('kind')=='online_prediction_chunk'))

    def advance(self,count=1):
        for _ in range(count):
            self.now+=.01;self.runtime.tick();time.sleep(.001)

    def until(self,predicate,count=500):
        for _ in range(count):
            if predicate():return
            self.advance()
            runner=self.runtime.owner.runner
            if runner and runner.status=='FAULT':self.fail(runner.reason)
        self.fail('CPU condition timed out: '+str(self.runtime.owner.runner.snapshot()))

    def request(self,target,*,key=None,gate=False):
        owner=self.runtime.owner
        request=dict(action='normal_mode',session_id=owner.runner.mode_session_id,runtime_nonce=owner.runtime_nonce,
            request_id=key or 'request-'+str(len(owner.runner.mode_receipts)),epoch=owner.epoch,
            mode='HOLD'if target in('END','DISCARD')else target,target=target)
        if gate:
            stamp=time.time_ns()
            request['recording_gate']={k:request[k]for k in('session_id','runtime_nonce','request_id','epoch')}
            request['recording_gate'].update(closed=True,clock_id='host_wall_time_ns',cutoff_ns=stamp,gate_generation=1,
                recorders=[dict(record_id=s,closed=True,clock_id='host_wall_time_ns',cutoff_ns=stamp,generation=1)for s in owner.runner.normal_config['recorder_ids']])
        result=self.runtime.submit(self.creds['inference'],request)
        return request,result

    def completed(self,request):
        key=request['request_id'];runner=self.runtime.owner.runner
        self.until(lambda:runner.mode_receipts[key]['status']in('completed','failed'))
        result=runner.mode_receipts[key];self.assertTrue(result['completed'],result)
        self.assertGreater(result['command_epoch'],request['epoch'])
        return result

    def finish_request(self, ending, *, sealed=True):
        owner=self.runtime.owner
        command=dict(action='normal_finish',session_id=owner.runner.mode_session_id,runtime_nonce=owner.runtime_nonce,
            request_id='finish-'+ending['request_id'],epoch=owner.epoch,ending_request_id=ending['request_id'],
            policy_digest=owner.runner.snapshot()['finish_capability']['policy_digest'])
        if sealed and owner.runner.normal_config['recording_required']:
            command['raw_sealed']={k:command[k]for k in('session_id','runtime_nonce','epoch','ending_request_id')}
            command['raw_sealed'].update(completed=True,recorders=[dict(record_id=s,completed=True,
                status='saved'if ending['target']=='END'else'discarded')for s in owner.runner.normal_config['recorder_ids']])
        return command

    def test_model_brake_hold_teach_explicit_hold_new_model_and_end(self):
        self.setup_modes();runner=self.runtime.owner.runner
        request,accepted=self.request('HOLD');self.assertFalse(accepted['completed'])
        duplicate=self.runtime.submit(self.creds['inference'],request);self.assertEqual(duplicate,accepted)
        with self.assertRaisesRegex(ValueError,'different content'):
            self.runtime.submit(self.creds['inference'],dict(request,target='END'))
        self.completed(request);self.assertEqual(runner.mode,'HOLD')
        anchor=runner.target_interpreter.reference.initial.copy();self.raw.position_bias[0]=.001
        self.advance(10);np.testing.assert_array_equal(runner.target_interpreter.reference.initial,anchor)
        self.assertEqual(runner.native_pending,[],'normal HOLD must not accumulate legacy prepared-window acknowledgements')
        request,_=self.request('TEACH');self.completed(request)
        self.assertEqual(runner.mode,'TEACH');self.assertTrue(self.raw.gain_calls)
        for row in self.raw.mode_gains.values():self.assertEqual(row['kp'],[0.]*6);self.assertEqual(row['gripper_kp'],5.)
        with self.assertRaisesRegex(ValueError,'explicit HOLD'):self.request('MODEL')
        self.raw.position_bias[0]=0
        request,_=self.request('HOLD');self.completed(request)
        old_requests=len(self.requests);request,_=self.request('MODEL');self.completed(request)
        self.assertGreater(len(self.requests),old_requests)
        self.assertGreater(self.requests[-1]['observation']['source_time'],request.get('submitted_s',0.))
        request,_=self.request('END');result=self.completed(request)
        self.assertEqual(result['target'],'END');self.assertEqual(runner.status,'READY')
        self.assertTrue(self.raw.is_connected)

    def test_recording_gate_required_and_fault_stop_never_waits_for_it(self):
        self.setup_modes(recording=True)
        with self.assertRaisesRegex(ValueError,'closed Raw gate'):self.request('HOLD')
        request,_=self.request('HOLD',gate=True)
        with self.assertRaisesRegex(ValueError,'never queued'):self.request('MODEL',key='during-transition')
        self.runtime.request_stop();self.advance(3)
        self.assertEqual(self.runtime.owner.runner.mode_receipts[request['request_id']]['status'],'failed')
        self.assertIsNone(self.raw.window)

    def test_confirmed_hold_retains_native_output_without_repeating_admission(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        self.setup_modes(recording=True,initial_mode='HOLD')
        runner=self.runtime.owner.runner
        for phase in ('initial','after_model'):
            before=self.raw.read_execution_output();epoch=self.runtime.owner.epoch
            packets=len(self.raw.packets)
            # Reproduce the observed failure if an idle HOLD tries to re-admit
            # an identical short window. Real feedback must continue throughout.
            with patch.object(self.raw,'queue_reference',side_effect=ReferenceWindowRejected(
                    'future reference expired during multi-arm admission'))as admission,\
                    patch.object(self.raw,'read_calibration_state',wraps=self.raw.read_calibration_state)as feedback:
                self.advance(500)
                self.assertNotIn('FAULT',self.runtime.owner.control.state.value,phase)
                admission.assert_not_called();self.assertGreater(feedback.call_count,400)
            after=self.raw.read_execution_output()
            self.assertEqual(before['q_sdk14'],after['q_sdk14'])
            self.assertEqual(after['dq_sdk14'],[0.]*14)
            self.assertEqual(before['gains'],after['gains'])
            self.assertEqual(epoch,self.runtime.owner.epoch);self.assertEqual(packets,len(self.raw.packets))
            request,_=self.request('MODEL');self.completed(request)
            request,_=self.request('HOLD',gate=True);self.completed(request)
        request,_=self.request('TEACH',gate=True);self.completed(request)
        request,_=self.request('HOLD',gate=True);self.completed(request)

    def test_prepared_initial_hold_does_not_infer_until_explicit_model_request(self):
        self.setup_modes(recording=True,initial_mode='HOLD');runner=self.runtime.owner.runner
        self.assertEqual(self.requests,[]);self.assertEqual(runner.mode,'HOLD')
        self.advance(50);self.assertEqual(self.requests,[])
        request,_=self.request('MODEL');self.completed(request)
        self.assertTrue(self.requests);self.assertEqual(runner.mode,'MODEL')

    def test_actual_control_runtime_client_routes_mode_and_completion_over_ipc(self):
        self.setup_modes();client,credential=self.setup_ipc();owner=self.runtime.owner
        command=dict(action='normal_mode',session_id=owner.runner.mode_session_id,runtime_nonce=owner.runtime_nonce,
            request_id='ipc-pause',epoch=owner.epoch,mode='HOLD',target='HOLD')
        accepted=client.submit(credential,command);self.assertFalse(accepted['completed'])
        self.completed(command)
        state=client._call('inference_state')['snapshot']
        self.assertEqual(state['mode'],'HOLD');self.assertTrue(state['mode_receipts']['ipc-pause']['completed'])
        self.assertEqual(client.submit(credential,command),state['mode_receipts']['ipc-pause'])

    def test_terminal_hold_is_not_a_noop_and_cannot_resume_finished_episode(self):
        for terminal in('END','DISCARD'):
            with self.subTest(terminal=terminal):
                self.setup_modes();request,_=self.request('HOLD');self.completed(request)
                runner=self.runtime.owner.runner
                with patch.object(runner,'_finish_data',wraps=runner._finish_data)as finish:
                    request,accepted=self.request(terminal);self.assertFalse(accepted['no_op'])
                    self.completed(request);self.assertTrue(finish.called)
                self.assertEqual(runner.status,'READY')
                with self.assertRaisesRegex(ValueError,'episode ended'):self.request('MODEL')
                self.cleanup()

    def test_native_linear_prefix_is_unchanged_at_every_sample_before_brake(self):
        self.setup_modes();old=deepcopy(self.raw.window)
        request,_=self.request('HOLD')
        self.until(lambda:self.runtime.owner.runner.normal_job['stage']=='BRAKE')
        job=self.runtime.owner.runner.normal_job;new=deepcopy(self.raw.window)
        times=np.linspace(new['host_times_s'][0],job['boundary_s'],101)
        for key in('q_sdk14','dq_sdk14','torque_bias_sdk14'):
            for j in range(14):
                expected=np.interp(times,old['host_times_s'],np.asarray(old[key])[:,j])
                actual=np.interp(times,new['host_times_s'],np.asarray(new[key])[:,j])
                np.testing.assert_allclose(actual,expected,rtol=0,atol=1e-12)
        self.completed(request)

    def test_missed_brake_preparation_deadline_faults_without_late_install(self):
        self.setup_modes();release=threading.Event();entered=threading.Event()
        from robot_workbench.motion_reference import prepare_brake_after
        def delayed(*args,**kwargs):
            entered.set();release.wait(2);return prepare_brake_after(*args,**kwargs)
        with patch('robot_workbench.motion_reference.prepare_brake_after',side_effect=delayed):
            request,_=self.request('HOLD');self.advance();self.assertTrue(entered.wait(1))
            self.advance(30);count=len(self.raw.packets)
            self.assertIn('FAULT',self.runtime.owner.control.state.value)
            self.assertIsNone(self.raw.window)
            release.set();self.advance(20)
            self.assertEqual(len(self.raw.packets),count)
            self.assertFalse(self.runtime.owner.runner.mode_receipts[request['request_id']]['completed'])

    def test_supply_exhaustion_uses_zero_endpoint_then_normal_hold(self):
        self.setup_modes();runner=self.runtime.owner.runner;runner.observe=lambda *args:None
        self.until(lambda:runner.mode=='HOLD'and runner.normal_job is None)
        self.assertNotIn('FAULT',self.runtime.owner.control.state.value)
        self.assertTrue(any(row['completed']for row in runner.mode_receipts.values()))

    def test_model_supply_error_requests_gate_and_cannot_bypass_it(self):
        self.setup_modes(recording=True);runner=self.runtime.owner.runner;self.model_error=True
        self.until(lambda:runner.snapshot()['transition_stage']=='AWAITING_RECORDING_GATE')
        self.assertNotIn('FAULT',self.runtime.owner.control.state.value)
        self.assertEqual(runner.mode,'MODEL')
        self.assertTrue(any(name=='inference_normal_pause_required'for name,value in self.events))
        with self.assertRaisesRegex(ValueError,'completed HOLD'):self.request('MODEL')
        request,_=self.request('HOLD',gate=True);self.completed(request)
        self.assertEqual(runner.mode,'HOLD')

    def test_native_admission_error_is_fault_not_model_supply_wait(self):
        from robot_workbench.sdk_process import SDKCommandRejected
        self.setup_modes(recording=True);request,_=self.request('HOLD',gate=True)
        with patch.object(self.raw,'queue_reference',side_effect=SDKCommandRejected('CPU native admission failed')):
            self.advance(25)
        runner=self.runtime.owner.runner
        self.assertIn('FAULT',self.runtime.owner.control.state.value)
        self.assertFalse(runner.mode_receipts[request['request_id']]['completed']);self.assertIsNone(self.raw.window)
        self.assertFalse(any(name=='inference_normal_pause_required'for name,value in self.events))

    def test_failed_model_resume_has_independent_current_epoch_hold_confirmation(self):
        self.setup_modes(recording=True,initial_mode='HOLD');runner=self.runtime.owner.runner;self.model_error=True
        request,_=self.request('MODEL')
        self.until(lambda:runner.mode_receipts[request['request_id']]['status']=='failed')
        result=runner.mode_receipts[request['request_id']]
        self.assertFalse(result['completed']);self.assertEqual(result['target'],'MODEL');self.assertEqual(result['mode'],'HOLD')
        self.assertEqual(result['error_class'],'prediction_supply');self.assertEqual(result['failure_phase'],'before_model_admission')
        self.assertTrue(result['retained_hold']['confirmed']);self.assertEqual(result['retained_hold']['command_epoch'],result['command_epoch'])
        self.assertEqual(result['command_epoch'],runner.snapshot()['command_epoch'])
        self.assertGreater(result['command_epoch'],request['epoch']);self.assertEqual(runner.mode,'HOLD')
        old=result['command_epoch'];self.runtime.request_stop();self.advance(2)
        self.assertGreater(runner.snapshot()['command_epoch'],old,'old hold confirmation is historical after Stop')
        self.assertNotEqual(runner.snapshot()['control_state'],'READY')

    def test_teach_can_end_without_model_resume(self):
        self.setup_modes();request,_=self.request('TEACH');self.completed(request)
        count=len(self.requests);request,_=self.request('END');self.completed(request)
        self.assertEqual(len(self.requests),count);self.assertEqual(self.runtime.owner.runner.status,'READY')

    def test_sealed_episode_finish_uses_four_original_stages_without_releasing_sdk(self):
        self.setup_modes(recording=True,finish_kind='dagger_reset_v1')
        ending,_=self.request('END',gate=True);self.completed(ending)
        with self.assertRaisesRegex(ValueError,'sealed'):
            self.runtime.submit(self.creds['inference'],self.finish_request(ending,sealed=False))
        command=self.finish_request(ending);accepted=self.runtime.submit(self.creds['inference'],command)
        self.assertFalse(accepted['completed']);runner=self.runtime.owner.runner
        self.until(lambda:runner.mode_receipts[command['request_id']]['status']in('completed','failed'),count=6000)
        result=runner.mode_receipts[command['request_id']]
        self.assertTrue(result['completed'],result);self.assertTrue(result['home_completed']);self.assertTrue(result['jaw_open_completed'])
        self.assertEqual(len(result['stages']),4);self.assertTrue(result['release_ready']);self.assertFalse(result['sdk_released'])
        self.assertTrue(self.raw.is_connected)
        targets=[r['execution_target']for r in result['stages']]
        self.assertEqual([q[2]for q in targets],[.03,0.,0.,0.])
        self.assertEqual([q[9]for q in targets],[0.,0.,.03,0.])
        self.assertAlmostEqual(targets[0][6],.082);self.assertAlmostEqual(targets[0][13],.03)
        self.assertAlmostEqual(targets[2][13],.082)
        self.assertTrue(all(r['status']=='READY'and r['native_window_submissions']>0 for r in result['stages']))
        self.assertEqual(runner.actual_submission_count,len(self.raw.packets))
        self.assertEqual(self.runtime.submit(self.creds['inference'],command),result)
        old=runner;epoch=self.runtime.owner.epoch
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=40))
        self.confirm('inference',review)
        with patch.object(self.raw,'begin',side_effect=AssertionError('next episode must reuse verified native HOLD')):
            self.until(lambda:self.runtime.owner.runner is not old and self.runtime.owner.runner.started is not None)
        self.assertNotEqual(self.runtime.owner.runner.execution_instance_id,old.execution_instance_id)
        self.assertTrue(any(name=='inference_native_hold_reused'for name,value in self.events))

    def test_policy_edit_after_end_preserves_native_hold_for_next_review(self):
        self.setup_modes(initial_mode='HOLD')
        ending,_=self.request('END');self.completed(ending)
        self.advance(40)
        previous=self.runtime.owner.runner
        reference=previous.target_interpreter
        output=self.raw.read_execution_output()['q_sdk14']
        stale=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=40))
        policy=self.runtime.control_policy_snapshot('inference')
        updated=self.runtime.update_control_policy(self.creds['inference'],dict(
            expected_revision=policy['revision'],changes={'measured.limit_rad_s':1.5}))
        self.assertEqual(self.raw.read_execution_output()['q_sdk14'],output)
        self.assertIs(self.runtime.owner.runner,previous)
        self.assertFalse(reference.cancelled)
        with self.assertRaises(ValueError):
            self.confirm('inference',stale)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=40))
        self.confirm('inference',review)
        self.until(lambda:self.runtime.owner.runner is not previous and self.runtime.owner.runner.mode=='HOLD')
        self.assertTrue(any(name=='inference_native_hold_reused'for name,_ in self.events))
        self.assertEqual(self.runtime.execution_calibration['control_policy']['policy_hash'],updated['policy_hash'])

    def test_native_home_keeps_feedback_and_returns_shared_hold(self):
        self.setup_modes(initial_mode='HOLD')
        request,_=self.request('HOME')
        result=self.completed(request)
        self.assertEqual(self.raw.home_calls,['left','right'])
        self.assertEqual(self.runtime.owner.runner.mode,'HOLD')
        self.assertEqual(self.requests,[])
        self.assertTrue(result['feedback']['native_home']['right']=='completed')
        self.assertTrue(self.runtime.owner.feedback_healthy)
        self.assertTrue(self.raw.is_connected)

    def test_manual_hold_accepts_feedback_margin_and_binds_legal_commands(self):
        self.setup_modes(initial_mode='HOLD')
        request,_=self.request('TEACH');self.completed(request)
        lower=self.raw.get_bimanual_joint_position_limits()[3][0]
        read=self.raw.read_calibration_state
        def dragged():
            state=read()
            if self.runtime.owner.runner.mode=='TEACH':
                self.raw.q[3]=state['qpos'][3]=lower-.004
            return state
        self.raw.read_calibration_state=dragged
        self.advance(35)
        request,_=self.request('HOLD');receipt=self.completed(request)
        self.assertGreaterEqual(receipt['native_output']['q_sdk14'][3],lower)
        self.assertTrue(all(receipt['native_output']['gains'][s]['gripper_kp']==5. for s in ('left','right')))
        self.assertEqual(self.requests,[])
        self.assertTrue(any(name=='inference_mode_transition'and row.get('stage')=='manual_hold_returned'
                            for name,row in self.events))

    def test_stop_during_manual_hold_wait_prevents_late_begin(self):
        self.setup_modes(initial_mode='HOLD')
        request,_=self.request('TEACH');self.completed(request)
        request,_=self.request('HOLD');self.advance(2)
        self.assertEqual(self.runtime.owner.runner.normal_job['stage'],'MANUAL_HOLD')
        self.assertTrue(all(self.raw.mode_gains[side]['gripper_kp']==5. for side in ('left','right')))
        with patch.object(self.raw,'begin',wraps=self.raw.begin) as begin:
            self.runtime.request_stop();self.advance(100)
            begin.assert_not_called()
        self.assertTrue(self.runtime.stop_receipt['stop_hold_return_confirmed'])

    def test_independent_stop_cancels_native_home_without_starting_other_arm(self):
        self.setup_modes(initial_mode='HOLD');request,_=self.request('HOME')
        self.advance(2);self.runtime.request_stop();self.advance(300)
        self.assertEqual(self.raw.home_calls,['left'])
        self.assertEqual(self.raw.native_home['left'],'cancelled')
        self.assertEqual(self.runtime.owner.runner.status,'STOPPED')
        self.assertTrue(self.runtime.stop_receipt['stop_hold_return_confirmed'])

    def test_maintenance_finish_preserves_jaws_and_timeout_never_claims_safe_finish(self):
        self.setup_modes(finish_kind='maintenance_home_v1');ending,_=self.request('END');self.completed(ending)
        before=self.raw.read_execution_output()['q_sdk14'];command=self.finish_request(ending)
        self.runtime.submit(self.creds['inference'],command)
        self.until(lambda:self.runtime.owner.runner.mode_receipts[command['request_id']]['completed'])
        result=self.runtime.owner.runner.mode_receipts[command['request_id']]
        self.assertTrue(result['home_completed']);self.assertFalse(result['jaw_open_completed']);self.assertEqual(len(result['stages']),1)
        np.testing.assert_allclose(np.asarray(result['native_output']['q_sdk14'])[[6,13]],np.asarray(before)[[6,13]])
        client,_=self.setup_ipc();client.begin_maintenance()
        actor=NS(action='release_control',require=lambda runtime:self.assertIs(runtime,client))
        released=client.control_release(actor)
        self.assertFalse(released['release_receipt']['sdk_close_returned'])
        self.assertFalse(self.runtime.tick());self.runtime.close()
        actual=client.refresh()['release_receipt']
        self.assertEqual(actual['status'],'released');self.assertTrue(actual['sdk_close_returned'])
        self.assertFalse(self.raw.is_connected)
        self.setup_modes(finish_kind='dagger_reset_v1',finish_timeout_s=.01)
        ending,_=self.request('DISCARD');self.completed(ending);command=self.finish_request(ending)
        self.runtime.submit(self.creds['inference'],command);self.advance(10)
        result=self.runtime.owner.runner.mode_receipts[command['request_id']]
        self.assertEqual(result['status'],'failed');self.assertFalse(result['release_ready']);self.assertFalse(result['home_completed'])
        self.assertTrue(self.raw.is_connected);self.assertIsNone(self.raw.window)

    def test_each_review_freezes_new_attempt_members_after_required_finish(self):
        self.setup_modes(recording=True,finish_kind='maintenance_home_v1',initial_mode='HOLD',per_episode=True)
        self.assertEqual(self.runtime.owner.runner.mode_session_id,'capture-1')
        request,_=self.request('MODEL');self.completed(request)
        ending,_=self.request('END',gate=True);self.completed(ending)
        with self.assertRaisesRegex(ValueError,'requires completed normal finish'):
            self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=40))
        finish=self.finish_request(ending);self.runtime.submit(self.creds['inference'],finish)
        self.until(lambda:self.runtime.owner.runner.mode_receipts[finish['request_id']]['completed'])
        with self.assertRaisesRegex(ValueError,'recorder membership'):
            self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=40))
        old=self.runtime.owner.runner;count=len(self.requests)
        context=dict(session_id='capture-2',recorder_ids=['robot-attempt-2','rgb-attempt-2'],clock_id='host_wall_time_ns')
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=40,recording_context=context))
        self.assertEqual(old.normal_config['recorder_ids'],['robot','rgb'],'review must not mutate old membership')
        self.confirm('inference',review)
        self.until(lambda:self.runtime.owner.runner is not old and self.runtime.owner.runner.mode=='HOLD'and self.runtime.owner.runner.normal_job is None)
        runner=self.runtime.owner.runner
        self.assertEqual(runner.mode_session_id,'capture-2');self.assertEqual(runner.normal_config['recorder_ids'],context['recorder_ids'])
        self.assertEqual(len(self.requests),count,'next episode starts in HOLD without an early model call')
        request,_=self.request('MODEL');self.completed(request)
        request=dict(action='normal_mode',session_id='capture-2',runtime_nonce=self.runtime.nonce,epoch=self.runtime.owner.epoch,
            request_id='old-membership',mode='HOLD',target='HOLD')
        gate=deepcopy(ending['recording_gate']);gate.update({k:request[k]for k in('session_id','runtime_nonce','epoch','request_id')})
        with self.assertRaisesRegex(ValueError,'active recorders'):
            self.runtime.submit(self.creds['inference'],dict(request,recording_gate=gate))


if __name__=='__main__':unittest.main()
