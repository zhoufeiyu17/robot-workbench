"""CPU policies exercise the real shared Owner and both runner strategies."""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests'), str(ROOT/'calibration'),
               str(ROOT/'calibration/infra'), str(ROOT/'calibration/tests'), str(ROOT/'rule_based_workbench')]
import test_unified_runtime as fixture
from robot_workbench.execution_settings import defaults, validate
from arx_env.joint_head_real_motion import MotionRunner, make_plan
from test_joint_head_real_motion import limits, feedback
from frozen_executor import FrozenExecutor
from test_frozen_executor import fixture as frozen_fixture


def setUpModule():
    global recording_fixture
    from test_recording_process import start_recording_fixture
    recording_fixture=start_recording_fixture()


def tearDownModule():
    from test_recording_process import stop_recording_fixture
    stop_recording_fixture(recording_fixture)


class SettingsTest(unittest.TestCase):
    setUp = fixture.UnifiedRuntimeTest.setUp
    cleanup = fixture.UnifiedRuntimeTest.cleanup
    create = fixture.UnifiedRuntimeTest.create
    advance = fixture.UnifiedRuntimeTest.advance
    preview = fixture.UnifiedRuntimeTest.preview
    confirm = fixture.UnifiedRuntimeTest.confirm

    def test_recording_policy_defaults_and_explicit_research_purpose(self):
        from robot_workbench.execution_settings import control_parameters,merge_control_policy,editable_policy_paths,validate_control_parameters
        for business,purpose in [('calibration','data_required'),('collection','data_required'),('inference','infer_debug'),('research',None)]:
            policy=control_parameters(defaults(business),business=business)
            self.assertEqual(policy['recording']['purpose_profile'],purpose)
            self.assertEqual(policy['recording']['resource_budget'],dict(event_queue_bytes=128*1024**2,message_bytes=512*1024))
            self.assertEqual('recording.purpose_profile' in editable_policy_paths(policy,business=business),business=='research')
        selected=merge_control_policy(policy,{'recording.purpose_profile':'data_required'},environment='fake',business='research')
        self.assertEqual(selected['recording']['purpose_profile'],'data_required')
        for value in (0,True,128*1024**2+1):
            with self.assertRaises(ValueError):merge_control_policy(selected,{'recording.resource_budget.event_queue_bytes':value},environment='fake',business='research')
        legacy=deepcopy(selected);legacy['recording']['resource_budget']['event_queue_bytes']=64*1024**2
        unchanged=validate_control_parameters(legacy,environment='fake',business='research')
        self.assertEqual(unchanged['recording']['resource_budget']['event_queue_bytes'],64*1024**2)
        edited=merge_control_policy(unchanged,{'arrival.jaw_mm':3.},environment='fake',business='research')
        self.assertEqual(edited['recording']['resource_budget']['event_queue_bytes'],64*1024**2)
        expanded=merge_control_policy(edited,{'recording.resource_budget.event_queue_bytes':128*1024**2},environment='fake',business='research')
        self.assertEqual(expanded['recording']['resource_budget']['event_queue_bytes'],128*1024**2)

    def test_recording_policy_uses_owner_store_and_legacy_edits_preserve_it(self):
        runtime,creds=self.create()
        updated=runtime.update_control_policy(creds['calibration'],dict(expected_revision=0,
            changes={'recording.resource_budget.event_queue_bytes':1024*1024}))
        self.assertEqual(updated['parameters']['recording']['resource_budget']['event_queue_bytes'],1024*1024)
        with self.assertRaises(ValueError):runtime.update_control_policy(creds['calibration'],dict(expected_revision=1,
            changes={'recording.purpose_profile':'infer_debug'}))
        old=runtime.execution_settings_snapshot('calibration')['settings'];old['arrival_jaw_mm']=3.
        runtime.update_execution_settings(creds['calibration'],old)
        current=runtime.control_policy_snapshot('calibration')
        self.assertEqual(current['parameters']['recording'],updated['parameters']['recording'])
        self.assertEqual(current['origins']['recording.resource_budget.event_queue_bytes'],'control_policy_api')
        stored=json.loads(runtime.execution_settings_path.read_text())
        self.assertEqual(stored['policies']['calibration']['recording'],updated['parameters']['recording'])

    def test_control_policy_atomic_fields_identity_and_legacy_projection(self):
        runtime, creds = self.create()
        initial = runtime.control_policy_snapshot('calibration')
        result = runtime.update_control_policy(creds['calibration'], dict(expected_revision=initial['revision'],
            changes={'measured.limit_rad_s':1.1, 'tracking.enabled':False, 'arrival.jaw_mm':3.}))
        self.assertEqual(result['revision'], initial['revision']+1)
        self.assertNotEqual(result['policy_hash'], initial['policy_hash'])
        self.assertNotIn('policy_id', result)
        self.assertEqual(result['origins']['arrival.jaw_mm'], 'control_policy_api')
        legacy = runtime.execution_settings_snapshot('calibration')['settings']
        self.assertEqual(legacy['speed_protection']['limit_rad_s'], 1.1)
        self.assertFalse(legacy['tracking_enabled']);self.assertEqual(legacy['arrival_jaw_mm'], 3.)
        result['parameters']['measured']['limit_rad_s']=99.
        self.assertEqual(runtime.control_policy_snapshot('calibration')['parameters']['measured']['limit_rad_s'],1.1)
        with self.assertRaisesRegex(ValueError,'revision'):
            runtime.update_control_policy(creds['calibration'],dict(expected_revision=initial['revision'],changes={'arrival.jaw_mm':4.}))
        self.assertEqual(json.loads(runtime.execution_settings_path.read_text())['schema'],'workbench.control-policy-store.v1')
        legacy['arrival_jaw_mm']=4.;runtime.update_execution_settings(creds['calibration'],legacy)
        self.assertEqual(runtime.control_policy_snapshot('calibration')['parameters']['arrival']['jaw_mm'],4.)

    def test_control_policy_failed_write_keeps_old_identity(self):
        runtime, creds=self.create();before=runtime.control_policy_snapshot('calibration')
        with patch.object(Path,'write_text',side_effect=OSError('full disk')):
            with self.assertRaises(OSError):
                runtime.update_control_policy(creds['calibration'],dict(expected_revision=before['revision'],changes={'tracking.jaw_mm':7.}))
        after=runtime.control_policy_snapshot('calibration')
        self.assertEqual((before['revision'],before['policy_hash']),(after['revision'],after['policy_hash']))
        self.assertFalse(runtime.settings_write_pending);self.assertFalse(runtime._policy_store_active)

    def test_control_policy_restores_and_preserves_other_business_defaults(self):
        runtime,creds=self.create()
        result=runtime.update_control_policy(creds['calibration'],dict(expected_revision=0,changes={'arrival.jaw_mm':5.}))
        runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
        runtime.tick();runtime.close()
        restored,creds=self.create()
        self.assertEqual(restored.control_policy_snapshot('calibration')['policy_hash'],result['policy_hash'])
        self.assertTrue(restored._policy_store_active)
        self.assertEqual(restored.execution_settings_snapshot('collection')['settings'],defaults('collection'))
        self.assertNotIn('collection',restored.execution_settings_policies)

    def test_control_policy_rejects_invalid_bootstrap_without_changing_state(self):
        from robot_workbench.execution_settings import control_parameters
        from robot_workbench.runtime import Runtime
        runtime,creds=self.create()
        before=runtime.control_policy_snapshot('calibration')
        with self.assertRaisesRegex(ValueError,'configuration object'):
            runtime.configure_inference([],lambda *args:None)
        policies={'calibration':control_parameters(defaults('calibration'))}
        policies['calibration']['execution']['execute_steps']=15
        stored=dict(schema='workbench.control-policy-store.v1',device_id=runtime.device_id,
                    environment=runtime.environment,revision=1,policies=policies,origins={})
        # Startup restore uses an unregistered Runtime; no device connection is needed.
        fresh=object.__new__(Runtime)
        fresh.lock=threading.RLock();fresh.clients={};fresh.owner=type('Owner',(),{'runner':None})()
        fresh.device_id=runtime.device_id;fresh.environment=runtime.environment
        with self.assertRaisesRegex(ValueError,'inference business'):
            fresh.restore_execution_settings(stored)
        self.assertEqual(runtime.control_policy_snapshot('calibration'),before)

    def test_control_policy_freezes_in_review_and_rejects_bad_fields(self):
        runtime,creds=self.create();before=runtime.control_policy_snapshot('calibration')
        for changes in ({'unknown':1},{'arrival.jaw_mm':-1},{'measured.limit_rad_s':float('nan')}):
            with self.assertRaises(ValueError):runtime.update_control_policy(creds['calibration'],dict(expected_revision=0,changes=changes))
        review=self.preview(runtime,creds['calibration']);runtime._guard_policy('calibration')
        with patch('robot_workbench.runtime.policy_hash',side_effect=AssertionError('execution admission must not hash policy')),\
                patch('robot_workbench.execution_settings.policy_hash',side_effect=AssertionError('bound policy must be prepared before Owner admission')):
            self.confirm(runtime,creds['calibration'],review);self.advance(runtime)
        frozen=runtime.execution_calibration['control_policy']
        self.assertEqual(frozen['policy_hash'],before['policy_hash'])
        self.assertEqual(frozen['parameters'],before['parameters'])
        runner=runtime.owner.runner
        self.assertIs(runner.guard,runtime.owner.control_guard)
        self.assertIs(runner.stationary.guard,runner.guard)
        self.assertTrue(all(row['policy_hash']==frozen['policy_hash']for row in runner.stationary.last_diagnostic['guard_decisions']))
        decision=runner.guard.evaluate(dict(rule_id='G25',signal_id=runner.plan.sha256,
            observed=dict(joint_errors_rad=[0.]*12,jaw_errors_mm=[0.,0.],exceeded_for_s=0.),source_time=1.,checked_time=1.),'tracking')
        self.assertEqual(decision['policy_hash'],frozen['policy_hash'])

    def test_actual_run_policy_binds_the_same_capture_and_guard_identity(self):
        from robot_workbench.operations import prepare_capture
        runtime,creds=self.create();review=self.preview(runtime,creds['calibration'])
        self.confirm(runtime,creds['calibration'],review);self.advance(runtime)
        frozen=runtime.execution_calibration['control_policy'];run_id=runtime.owner.execution_instance_id
        sources=[dict(stream_id='feedback',kind='feedback',source_instance_id=runtime.nonce,
            source_clock_id='cpu-clock',units='rad',coordinate_frame='joint',nominal_hz=100.,required=True)]
        session,result=prepare_capture(runtime.execution_settings_path.parent/'captures',request_id='cpu-run-capture',
            run_id=run_id,business_kind='calibration',environment='fake',policy=frozen,source_descriptors=sources,
            still_current=lambda:runtime.owner.execution_instance_id==run_id)
        try:
            self.assertTrue(result['ready'])
            self.assertEqual(result['decision']['policy_hash'],frozen['policy_hash'])
            self.assertEqual(result['binding']['policy_hash'],frozen['policy_hash'])
            self.assertEqual(result['binding']['run_id'],run_id)
        finally:session.close()

    def test_control_policy_inference_length_updates_bound_configuration(self):
        runtime,creds=self.create();creds['inference']=runtime.register_business('inference')
        runtime.handoff(creds['calibration'],'inference')
        runtime.configure_inference(dict(model_id='cpu-fixture',duration_s=40.,execute_steps=15),lambda *args:None)
        state=runtime.control_policy_snapshot('inference')
        result=runtime.update_control_policy(creds['inference'],dict(expected_revision=state['revision'],changes={
            'execution.execute_steps':30,'execution.prepare_drift_joint_rad':.003,'execution.prepare_drift_jaw_mm':1.}))
        self.assertEqual(result['parameters']['execution']['execute_steps'],30)
        self.assertEqual(runtime.inference_configuration['execute_steps'],30)
        self.assertEqual(runtime.inference_configuration['prepare_drift_joint_rad'],.003)
        self.assertEqual(runtime.inference_configuration['prepare_drift_jaw_mm'],1.)
        runtime.configure_inference(dict(model_id='cpu-fixture',duration_s=40.,execute_steps=15),lambda *args:None)
        self.assertEqual(runtime.inference_configuration['execute_steps'],30)

    def test_inference_length_compatibility_uses_owner_store_without_changing_holder_policy(self):
        from types import SimpleNamespace
        from robot_workbench.inference_workflow import InferenceWorkflow
        runtime,creds=self.create();creds['inference']=runtime.register_business('inference')
        flow=InferenceWorkflow.__new__(InferenceWorkflow)
        flow.lock=threading.RLock();flow.execute_steps=45;flow.execution_revision=0
        flow.reference_planning=None;flow.finite_chunk_count=None;flow.generation=0
        flow.device=SimpleNamespace(runtime=runtime,credentials=creds)
        flow.snapshot=lambda:dict(busy=False)
        flow.invalidate=lambda:None
        self.assertEqual(flow.execution_settings()['execute_steps'],45)
        before=runtime.execution_settings_snapshot('calibration')['settings']
        result=flow.set_execution_steps(30,0)
        self.assertEqual(result['execute_steps'],30)
        self.assertEqual(runtime.control_policy_snapshot('inference')['parameters']['execution']['execute_steps'],30)
        self.assertEqual(runtime.execution_settings_snapshot('calibration')['settings'],before)
        self.assertEqual(runtime.holder,'calibration')
        with self.assertRaises(PermissionError):
            runtime.update_control_policy(creds['inference'],dict(expected_revision=1,changes={'measured.limit_rad_s':2.}))

    def test_policy_planning_projection_is_normalized_and_keeps_one_duration_source(self):
        from test_variable_inference_steps import config
        from robot_workbench.execution_settings import control_parameters,merge_control_policy,policy_hash
        from robot_workbench.runtime import Runtime
        original=config(15)
        parameters=control_parameters(defaults('inference'),configuration=original,business='inference')
        changed=merge_control_policy(parameters,{'execution.execute_steps':30,
            'execution.prepare_drift_joint_rad':.003,'execution.prepare_drift_jaw_mm':1.,
            'planning.velocity_limit_rad_s':.5,'planning.acceleration_limit_rad_s2':20,
            'planning.jaw_velocity_limit_mm_s':20},environment='fake')
        runtime=object.__new__(Runtime)
        derived=runtime._configuration_from_policy(original,changed)
        self.assertEqual(derived['reference_planning']['max_execution_duration_s'],4.)
        self.assertEqual(derived['normal_modes']['velocity_limit'],.5)
        self.assertEqual(derived['normal_modes']['acceleration_limit'],20.)
        self.assertEqual(derived['prepare_drift_joint_rad'],.003)
        self.assertEqual(derived['prepare_drift_jaw_mm'],1.)
        self.assertEqual(original['reference_planning']['max_execution_duration_s'],2.)
        floats=merge_control_policy(parameters,{'planning.acceleration_limit_rad_s2':20.0},environment='fake')
        integers=merge_control_policy(parameters,{'planning.acceleration_limit_rad_s2':20},environment='fake')
        self.assertEqual(policy_hash(floats),policy_hash(integers))
        with self.assertRaises(ValueError):
            merge_control_policy(parameters,{'arrival.jaw_mm':3.,'planning.velocity_limit_rad_s':2.},environment='fake')
        self.assertEqual(parameters['arrival']['jaw_mm'],2.)

    def test_common_guard_uses_policy_and_preserves_boundaries_and_times(self):
        from robot_workbench.execution_settings import Guard,control_parameters,policy_hash
        parameters=control_parameters(defaults('inference'),configuration=dict(maximum_observation_age_s=.5),business='inference')
        policy=dict(revision=7,policy_hash=policy_hash(parameters),parameters=parameters)
        for rule,limit in [('G02',.5),('G03',.1),('G04',.1),('G11',math.radians(2.)),('G12',2.),('G14',.8)]:
            signal=dict(rule_id=rule,signal_id='cpu',observed=limit,source_time=10.,checked_time=10.5,epoch=3)
            passed=Guard.evaluate(signal,'submission',policy)
            self.assertEqual(passed['verdict'],'pass')
            self.assertEqual((passed['revision'],passed['policy_hash']),(7,policy['policy_hash']))
            self.assertEqual((passed['source_time'],passed['checked_time']),(10.,10.5))
            rejected=Guard.evaluate(dict(signal,observed=limit+1e-9),'submission',policy)
            self.assertEqual(rejected['verdict'],'reject')
            if rule not in ('G02','G03','G04'):self.assertEqual(Guard.evaluate(dict(signal,observed=-limit),'submission',policy)['verdict'],'pass')
        future=dict(signal,rule_id='G02',observed=-1e-9)
        self.assertEqual(Guard.evaluate(future,'submission',policy)['verdict'],'reject')
        disabled=deepcopy(parameters);disabled['measured']['enabled']=False
        disabled_policy=dict(revision=8,policy_hash=policy_hash(disabled),parameters=disabled)
        self.assertEqual(Guard.evaluate(dict(signal,observed=9.),'execution',disabled_policy)['verdict'],'observe')
        for value in (float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):Guard.evaluate(dict(signal,observed=value),'execution',policy)

    def test_trajectory_guards_use_frozen_limits_steps_and_derived_duration(self):
        from test_variable_inference_steps import config
        from robot_workbench.execution_settings import Guard,control_parameters,policy_hash
        for steps in (1,15,60):
            parameters=control_parameters(defaults('inference'),configuration=config(steps),business='inference')
            policy=dict(revision=4,policy_hash=policy_hash(parameters),parameters=parameters)
            for rule,limit in [('G13',1.),('G15',40.),('G16',40.),('G21',steps),('G22',steps/30.*4)]:
                signal=dict(rule_id=rule,signal_id='cpu-reference',observed=limit,source_time=10.,checked_time=10.1)
                result=Guard.evaluate(signal,'submission',policy)
                self.assertEqual(result['verdict'],'pass')
                self.assertEqual(result['limit'],limit)
                self.assertEqual(result['policy_hash'],policy['policy_hash'])
                for value in (limit+1e-8,-1):
                    self.assertEqual(Guard.evaluate(dict(signal,observed=value),'submission',policy)['action'],'REJECT_PLAN')
                if rule=='G21':
                    self.assertEqual(Guard.evaluate(dict(signal,observed=steps-.5),'submission',policy)['verdict'],'reject')
            self.assertEqual(policy['parameters']['execution']['execute_steps'],steps)

    def test_bound_guard_freezes_capabilities_and_handles_vectors_with_one_identity(self):
        from robot_workbench.execution_settings import Guard,control_parameters,policy_hash
        parameters=control_parameters(defaults('calibration'));parameters['tracking']['duration_s']=.2
        policy=dict(revision=3,policy_hash=policy_hash(parameters),parameters=parameters)
        original_policy=deepcopy(policy)
        context=dict(environment='physical',joint_limits=[[-1.,1.]]*12,feedback_timeout_s=.1,starvation_margin_s=.015)
        bound=Guard.bind(policy,context)
        context['joint_limits'][0]=[-9.,9.];parameters['arrival']['tolerances_rad'][0]=9.
        def check(rule,observed):return bound.evaluate(dict(rule_id=rule,signal_id='cpu-reference',observed=observed,
            source_time=10.,checked_time=10.05,run_id='cpu-run'),'feedback')
        self.assertEqual(check('G18',[1.]*12)['verdict'],'pass')
        import numpy as np
        self.assertEqual(check('G18',[np.float64(1.)]*12)['verdict'],'pass')
        self.assertEqual(check('G18',[1.00001]+[0.]*11)['action'],'REJECT_PLAN')
        self.assertEqual(check('G19',[1.009]+[0.]*11)['verdict'],'pass')
        self.assertEqual(check('G19',[1.011]+[0.]*11)['verdict'],'reject')
        self.assertEqual(check('G20',[1.02]+[0.]*11)['verdict'],'pass')
        self.assertEqual(check('G20',[1.+math.radians(5.01)]+[0.]*11)['action'],'FAULT_STOP')
        self.assertEqual(check('G28',.10001)['action'],'FAULT_STOP')
        self.assertEqual(check('G29',.015)['verdict'],'pass')
        self.assertEqual(check('G29',.014)['action'],'FAULT_STOP')
        errors=dict(joint_errors_rad=[.09]+[0.]*11,jaw_errors_mm=[0.,0.],exceeded_for_s=.199)
        self.assertEqual(check('G25',errors)['verdict'],'observe')
        self.assertEqual(check('G25',dict(errors,exceeded_for_s=.2))['action'],'FAULT_STOP')
        arrival=check('G26',dict(joint_errors_rad=[math.radians(2.01)]+[0.]*11,jaw_errors_mm=[0.,0.]))
        self.assertEqual(arrival['action'],'WAIT')
        self.assertEqual(arrival['observed']['exceeded_joint_indices'],[0])
        self.assertEqual(arrival['revision'],3)
        self.assertEqual(arrival['policy_hash'],policy['policy_hash'])
        for observed in ([0.]*11,[float('nan')]*12,[True]*12):
            with self.assertRaises(ValueError):check('G18',observed)
        with self.assertRaisesRegex(ValueError,'missing'):
            Guard.bind(original_policy,{}).evaluate(dict(rule_id='G18',signal_id='cpu',observed=[0.]*12,source_time=1.,checked_time=1.),'feedback')
        identity={'worker_identity':{'tokenizer_sha256':'a'*64,'epoch':10}}
        model=Guard.bind(original_policy,dict(expected_identity=identity))
        self.assertEqual(model.evaluate(dict(rule_id='G01',signal_id='model',observed=identity,source_time=1.,checked_time=1.),'model')['verdict'],'pass')
        for key in ('token','session_token','password','authorization','api_key'):
            with self.assertRaisesRegex(ValueError,'credentials'):Guard.bind(original_policy,dict(expected_identity={key:'private'}))

    def test_stationary_watchdog_and_legacy_profiles_keep_their_original_boundaries(self):
        from robot_workbench.execution_settings import Guard,control_parameters,policy_hash,STATIONARY_PROFILE,LEGACY_REFERENCE_LIMITS
        parameters=control_parameters(defaults())
        policy=dict(revision=8,policy_hash=policy_hash(parameters),parameters=parameters)
        guard=Guard.bind(policy,{})
        def check(rule,value,phase):return guard.evaluate(dict(rule_id=rule,signal_id='cpu-profile',observed=value,source_time=1.,checked_time=1.1),phase)
        self.assertEqual(check('G27',.1,'stationary_gap')['verdict'],'pass')
        self.assertEqual(check('G27',.10001,'stationary_gap')['action'],'WAIT')
        self.assertEqual(check('G27',.3-1e-9,'stationary_window')['verdict'],'pass')
        self.assertEqual(check('G27',.299,'stationary_window')['verdict'],'reject')
        span=[STATIONARY_PROFILE['joint_span_rad']]*14;span[6]=span[13]=2.
        self.assertEqual(check('G27',span,'stationary_span')['verdict'],'pass')
        span[13]+=1e-8
        self.assertEqual(check('G27',span,'stationary_span')['action'],'WAIT')
        self.assertEqual(check('G31',LEGACY_REFERENCE_LIMITS,'legacy_reference')['verdict'],'pass')
        self.assertEqual(check('G31',dict(LEGACY_REFERENCE_LIMITS,velocity_limit_rad_s=.25000001),'legacy_reference')['action'],'REJECT_PLAN')
        facts=dict(monitored=True,age_s=.1,fault_latched=False)
        self.assertEqual(check('G30',facts,'native_watchdog')['verdict'],'pass')
        for changed in (dict(age_s=None),dict(age_s=.10001),dict(fault_latched=True),dict(age_s=-.001)):
            decision=check('G30',dict(facts,**changed),'native_watchdog')
            self.assertEqual(decision['action'],'FAULT_STOP');self.assertEqual(decision['policy_hash'],policy['policy_hash'])
        self.assertEqual(check('G30',dict(facts,monitored=False,age_s=None),'native_watchdog')['verdict'],'pass')
        for rule,value,phase in [('G27',[float('nan')]*14,'stationary_span'),('G30',dict(facts,age_s=float('nan')),'native_watchdog'),('G31',dict(LEGACY_REFERENCE_LIMITS,velocity_limit_rad_s=True),'legacy_reference')]:
            with self.assertRaises(ValueError):check(rule,value,phase)

    def test_old_input_policy_migrates_and_projects_to_the_same_run_configuration(self):
        from robot_workbench.execution_settings import control_parameters,validate_control_parameters,merge_control_policy
        from robot_workbench.runtime import Runtime
        old=control_parameters(defaults('inference'),business='inference')
        old['observation']={'maximum_age_s':.5}
        migrated=validate_control_parameters(old,environment='fake',business='inference')
        self.assertEqual(migrated['observation'],dict(maximum_age_s=.5,state_max_age_s=.1,max_camera_span_s=.1))
        edited=merge_control_policy(migrated,{'observation.state_max_age_s':.05,'observation.max_camera_span_s':.03},environment='fake',business='inference')
        frozen=Runtime._configuration_from_policy(object.__new__(Runtime),dict(model_id='cpu',duration_s=40),edited)
        self.assertEqual((frozen['state_max_age_s'],frozen['max_camera_span_s']),(.05,.03))
        self.assertEqual(old['observation'],{'maximum_age_s':.5})
        for value in (0,.100001,True,float('nan')):
            with self.assertRaises(ValueError):merge_control_policy(migrated,{'observation.state_max_age_s':value},environment='fake',business='inference')

    def test_recording_purpose_decision_does_not_weaken_control_rules(self):
        from robot_workbench.execution_settings import Guard,control_parameters,policy_hash
        parameters=control_parameters(defaults('inference'),business='inference')
        for purpose in ('data_required','infer_debug'):
            parameters['recording']['purpose_profile']=purpose
            policy=dict(revision=1,policy_hash=policy_hash(parameters),parameters=deepcopy(parameters))
            for rule,action in [('G35','REJECT_INPUT'),('G38','BRAKE_TO_HOLD')]:
                signal=dict(rule_id=rule,signal_id='recorder',observed=1,source_time=1.,checked_time=1.01)
                decision=Guard.evaluate(signal,'capture',policy)
                self.assertEqual(decision['action'],action if purpose=='data_required' else 'CONTINUE')
                self.assertEqual(decision['severity'],'error' if purpose=='data_required' else 'warning')
                self.assertEqual(Guard.evaluate(dict(signal,observed=0),'capture',policy)['verdict'],'pass')
                self.assertEqual(Guard.evaluate(signal,'seal',policy)['action'],'CONTINUE')
                with self.assertRaises(ValueError):Guard.evaluate(dict(signal,observed=-1),'capture',policy)
            signal.update(rule_id='G14',observed=.9)
            self.assertEqual(Guard.evaluate(signal,'execution',policy)['action'],'FAULT_STOP')
            bound=Guard.bind(policy,{})
            for rule in ('G33','G36','G37'):
                fact=dict(rule_id=rule,signal_id='recorder',source_time=1.,checked_time=1.1,
                    observed=dict(failed_conditions=1,receipt_ref='manifest.json'))
                active=bound.evaluate(fact,'recording')
                self.assertEqual(active['action'],'BRAKE_TO_HOLD'if purpose=='data_required'and rule!='G33'else'CONTINUE')
                sealed=bound.evaluate(fact,'seal')
                self.assertEqual((sealed['verdict'],sealed['action']),('observe','CONTINUE'))
                fact['observed']['failed_conditions']=0
                self.assertEqual(bound.evaluate(fact,'seal')['verdict'],'pass')

    def test_stalled_settings_storage_does_not_block_tick_or_stop_or_admit_motion(self):
        for phase in ('write', 'replace'):
            with self.subTest(phase=phase):
                runtime, creds = self.create(device='disk-'+phase)
                review = self.preview(runtime, creds['calibration'])
                settings = runtime.execution_settings_snapshot('calibration')['settings']
                settings['arrival_jaw_mm'] = 7.
                entered, release = threading.Event(), threading.Event()
                results, errors = [], []
                def stalled(*args, **kwargs):
                    entered.set()
                    if not release.wait(3):raise TimeoutError('test storage gate')
                    return original(*args, **kwargs)
                def update():
                    try:results.append(runtime.update_execution_settings(creds['calibration'],settings))
                    except BaseException as error:errors.append(error)
                from robot_workbench import runtime as module
                original = Path.write_text if phase=='write' else module.os.replace
                target, name = (Path, 'write_text') if phase=='write' else (module.os, 'replace')
                with patch.object(target,name,stalled):
                    worker=threading.Thread(target=update);worker.start()
                    try:
                        self.assertTrue(entered.wait(1))
                        self.assertTrue(runtime.settings_write_pending)
                        self.advance(runtime)
                        self.assertTrue(runtime.owner.feedback_healthy)
                        with self.assertRaisesRegex(ValueError,'配置正在保存'):
                            self.confirm(runtime,creds['calibration'],review)
                        with self.assertRaisesRegex(ValueError,'配置正在保存'):
                            runtime.update_execution_settings(creds['calibration'],settings)
                        with self.assertRaisesRegex(ValueError,'配置正在保存'):
                            runtime.begin_maintenance()
                        runtime.request_stop();self.advance(runtime)
                        self.assertEqual(runtime.owner.control.state,runtime.owner.states.STOPPED)
                        self.assertFalse(release.is_set(),'control must complete while storage is still blocked')
                    finally:release.set();worker.join(3)
                self.assertFalse(worker.is_alive());self.assertFalse(errors,errors)
                self.assertEqual(len(results),1);self.assertFalse(runtime.settings_write_pending)
                self.assertEqual(runtime.owner.control.state,runtime.owner.states.STOPPED)
                self.assertEqual(json.loads(runtime.execution_settings_path.read_text()),runtime.export_execution_settings())
                with self.assertRaisesRegex(ValueError,'unknown or consumed'):
                    self.confirm(runtime,creds['calibration'],review)

    def test_failed_settings_write_keeps_previous_policy_and_releases_reservation(self):
        runtime,creds=self.create()
        previous=runtime.export_execution_settings()
        settings=runtime.execution_settings_snapshot('calibration')['settings']
        settings['arrival_jaw_mm']=7.
        with patch.object(Path,'write_text',side_effect=OSError('injected full disk')):
            with self.assertRaisesRegex(OSError,'full disk'):
                runtime.update_execution_settings(creds['calibration'],settings)
        self.assertEqual(runtime.export_execution_settings(),previous)
        self.assertFalse(runtime.settings_write_pending)
        self.preview(runtime,creds['calibration'])

    def test_update_preserves_defaults_revokes_reviews_and_persists_without_motion(self):
        runtime, creds = self.create()
        runtime.owner.arm_tolerances = [.007+i*.001 for i in range(12)]
        settings = runtime.execution_settings_snapshot('calibration')['settings']
        self.assertEqual(settings['arrival_tolerances_rad'], runtime.owner.arm_tolerances)
        self.assertFalse(runtime.execution_settings_snapshot('collection')['settings']['tracking_enabled'])
        preview = self.preview(runtime, creds['calibration'])
        before = deepcopy(runtime.owner.backend.events)
        settings.update(arrival_enabled=False, tracking_enabled=False)
        result = runtime.update_execution_settings(creds['calibration'], settings)
        self.assertEqual(result['revision'], 1)
        self.assertEqual(runtime.owner.backend.events, before)
        self.assertFalse(runtime.reviewed_plans)
        self.assertEqual(json.loads(runtime.execution_settings_path.read_text()), runtime.export_execution_settings())
        with self.assertRaisesRegex(ValueError, 'unknown or consumed'):
            self.confirm(runtime, creds['calibration'], preview)
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        self.advance(runtime)
        self.assertEqual(runtime.owner.runner.error_settings, settings)
        self.assertEqual(runtime.execution_calibration['execution_settings']['settings'], settings)
        with self.assertRaisesRegex(ValueError, 'motion or capture'):
            runtime.update_execution_settings(creds['calibration'], settings)
        runtime.submit(creds['collection'], {'action':'stop'})
        self.advance(runtime)
        self.assertEqual(runtime.owner.runner.status, 'STOPPED')

    def test_authentication_backend_support_and_bad_values(self):
        runtime, creds = self.create()
        settings = defaults()
        with self.assertRaises(PermissionError):
            runtime.update_execution_settings(creds['collection'], settings)
        settings['fake_error']['enabled'] = True
        with self.assertRaisesRegex(ValueError, 'does not support'):
            runtime.update_execution_settings(creds['calibration'], settings)
        self.assertEqual(runtime.execution_settings_revision, 0)
        with self.assertRaisesRegex(ValueError, 'only in fake'):
            validate(settings, environment='physical')
        for changes in ({'arrival_enabled':1}, {'tracking_joint_rad':float('nan')},
                        {'arrival_tolerances_rad':[.01]*11}, {'tracking_duration_s':-1}):
            with self.assertRaises(ValueError):
                validate(dict(defaults(), **changes))

    def test_persistent_policy_restores_and_business_handoff_keeps_other_default(self):
        runtime,creds=self.create()
        settings=defaults();settings.update(arrival_enabled=False,tracking_enabled=False,arrival_jaw_mm=7.)
        runtime.update_execution_settings(creds['calibration'],settings)
        runtime.handoff(creds['calibration'],'collection')
        self.assertEqual(runtime.execution_settings_snapshot('collection')['settings'], defaults('collection'))
        runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
        runtime.tick();runtime.close()
        restored,creds=self.create()
        self.assertEqual(restored.execution_settings_snapshot('calibration')['settings'],settings)
        self.assertEqual(restored.execution_settings_revision,1)
        with self.assertRaisesRegex(ValueError,'startup-only'):
            restored.restore_execution_settings(restored.export_execution_settings())

    def test_untouched_calibration_keeps_legacy_passive_guards_until_explicit_save(self):
        for custom in (False, True):
            runtime, creds = self.create(device='custom' if custom else 'untouched')
            if custom:
                runtime.update_execution_settings(creds['calibration'], defaults())
            state = runtime.execution_settings_snapshot('calibration')
            self.assertEqual(state['legacy_passive_guards'], not custom)
            self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
            self.advance(runtime, 35)
            self.assertEqual(runtime.owner.runner.configured_error_policy, custom)
            self.assertEqual(runtime.execution_calibration['execution_settings']['legacy_passive_guards'], not custom)
            runtime.owner.backend.q[7] = math.radians(2.01)
            self.advance(runtime)
            if custom:
                self.assertNotEqual(runtime.owner.runner.status, 'FAULT')
            else:
                self.assertEqual(runtime.owner.runner.status, 'FAULT')
                self.assertIn('inactive arm drift', runtime.owner.runner.reason)

    def test_manual_no_threshold_finishes_with_real_residual_but_stale_feedback_still_faults(self):
        q = [0.]*14
        target = q[:]; target[0] = .2
        plan = make_plan('both', q, target, limits(), kind='workbench', created_at_s=8500., duration_s=2.)
        settings = defaults(); settings.update(arrival_enabled=False, tracking_enabled=False)
        sent=[]
        runner = MotionRunner(plan, begin=lambda q:q, send=sent.append, stop=lambda reason:None,
                              execution_settings=settings)
        runner.start(8500., feedback(8500., q))
        for i in range(1, 50):
            now = 8500.+i*.05
            runner.tick(now, feedback(now, q))
        self.assertEqual(runner.status, 'READY', runner.reason)
        self.assertAlmostEqual(sent[-1][0], .2)
        self.assertGreater(runner.tracking_exceeded_samples, 0)
        self.assertAlmostEqual(runner.target_error['arm_rad']['left_J1'], -.2)
        self.assertEqual(runner.tick(now+.05, feedback(now-.2, q)), 'FAULT')
        self.assertIn('older than', runner.reason)

    def test_configured_tracking_and_arrival_are_independent(self):
        for arrival, tracking, expected in ((False,False,'READY'), (True,False,'FAULT'), (False,True,'FAULT')):
            q=[0.]*14; target=q[:];target[0]=.15
            plan=make_plan('both',q,target,limits(),kind='workbench',created_at_s=8500.,duration_s=2.)
            settings=defaults(); settings.update(arrival_enabled=arrival,tracking_enabled=tracking,
                                                tracking_joint_rad=.02,tracking_duration_s=.1)
            runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=lambda why:None,execution_settings=settings)
            runner.start(8500.,feedback(8500.,q))
            for n in range(1,120):runner.tick(8500.+n*.05,feedback(8500.+n*.05,q))
            self.assertEqual(runner.status,expected,runner.reason)

    def test_capture_waits_for_stationarity_without_requiring_arrival_and_queue_continues(self):
        runtime,creds=self.create()
        settings=defaults();settings.update(arrival_enabled=False,tracking_enabled=False)
        runtime.update_execution_settings(creds['calibration'],settings)
        waypoints=[dict(q=[.03]*12,speeds=[.1]*12,capture=True,sample_id='first'),
                   dict(q=[.06]*12,speeds=[.1]*12,capture=False)]
        review=runtime.submit(creds['calibration'],dict(action='preview_workbench',waypoints=waypoints))
        self.confirm(runtime,creds['calibration'],review)
        backend=runtime.owner.backend._backend
        def lag(side,q):
            # Real feedback remains far from command; hold the actual zero pose.
            backend.events.append(('send',side,list(q)))
        with patch.object(backend,'send',side_effect=lag):
            for _ in range(300):
                self.advance(runtime)
                if runtime.owner.capture_wait:break
            self.assertIsNotNone(runtime.owner.capture_wait,runtime.owner.snapshot())
            self.assertAlmostEqual(runtime.owner.runner.target_error['arm_rad']['left_J1'],-.03)
            old=runtime.owner.capture_wait['ready_wall_ns']
            # Motion after READY revokes the capture window despite observation mode.
            for n in range(3):
                backend.q[0]+=math.radians(2.01)
                self.advance(runtime)
            self.assertIsNone(runtime.owner.capture_wait)
            for _ in range(100):
                self.advance(runtime)
                if runtime.owner.capture_wait:break
            self.assertGreater(runtime.owner.capture_wait['ready_wall_ns'],old)
            runtime.submit(creds['calibration'],dict(action='capture_ack',owner_pid=runtime.owner.cache['pid'],
                command_epoch=runtime.owner.epoch,sample_id='first',capture_ready_wall_ns=runtime.owner.capture_wait['ready_wall_ns']))
            for _ in range(300):self.advance(runtime)
            self.assertEqual(runtime.owner.runner.status,'READY',runtime.owner.runner.reason)
            self.assertFalse(runtime.owner.sequence_remaining)
            self.assertIsNone(runtime.owner.capture_sample)

    def test_frozen_tracking_duration_and_endpoint_policy(self):
        source,plan=frozen_fixture()
        for arrival,tracking,expected in ((False,False,'completed'),(True,False,'failed'),(False,True,'failed')):
            settings=defaults('collection');settings.update(arrival_enabled=arrival,tracking_enabled=tracking,
                tracking_joint_rad=.001,tracking_duration_s=.02,arrival_tolerances_rad=[.001]*12)
            follower=FrozenExecutor(plan,source,sink=lambda q:None,gate=lambda:True,stop=lambda why:None,
                                    execution_policy='observation',execution_settings=settings)
            measured=deepcopy(plan['ticks'][0]['target'])
            state=lambda t:dict(sample_monotonic_s=t,state=measured)
            follower.start(10.,state(10.))
            for n in range(1,400):
                result=follower.tick(10.+n*.01,state(10.+n*.01))
                if result['status'] in ('completed','failed'):break
            self.assertEqual(result['status'],expected,result)
            self.assertGreater(result['tracking_exceeded_samples'],0)
            self.assertEqual(result['execution_settings'],settings)


if __name__ == '__main__':
    unittest.main()
