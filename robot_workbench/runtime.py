"""One managed device owner; server-side business credentials, no HTTP or SDK loader.

The embedding service supplies the reviewed backend factory/initialization and
runs tick() on its sole control thread. A local lock excludes cooperating owners,
not arbitrary external CAN programs. SDK return evidence is not firmware ACK.
"""
from contextlib import contextmanager
from copy import copy, deepcopy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
from .execution_settings import (defaults as default_execution_settings, validate as validate_execution_settings,
                                 measured_joint_speed, JointSpeedProtectionError,validate_study)
from .execution_settings import (control_parameters, execution_settings_from_policy, editable_policy_paths,
                                 merge_control_policy, validate_control_parameters, policy_hash, Guard)
import queue
import secrets
import stat
import threading
import time


BUSINESSES = {'calibration', 'collection', 'inference', 'research'}
CALIBRATION_PREVIEWS = {'preview', 'preview_home', 'preview_arm_home', 'preview_jaw_target',
                        'preview_both_home', 'preview_workbench'}
COLLECTION_PREVIEWS = {'preview_frozen', 'preview_start_alignment'}
INFERENCE_PREVIEWS = {'preview_inference','preview_operator_home','preview_operator_hold'}
CONTROL_OUTPUTS={'begin','send','begin_collection','enable_collection_arm','queue_reference',
                 'configure_motion_gains','set_execution_gains','bind_execution_context','set_damping_hold','close','start_native_home'}


class _Backend:
    """Track entry/return of real backend calls without locking around native IO."""
    def __init__(self, backend, runtime):
        object.__setattr__(self, '_backend', backend)
        object.__setattr__(self, '_runtime', runtime)

    def __setattr__(self, name, value):
        setattr(self._backend, name, value)

    def __getattr__(self, name):
        value = getattr(self._backend, name)
        if name not in {'connect', 'close', 'begin', 'send', 'set_damping_hold',
                         'read_calibration_state', 'read_execution_output', 'set_execution_gains', 'get_bimanual_joint_position_limits', 'read_gains', 'enable_collection_arm','queue_reference','configure_motion_gains','bind_execution_context','start_native_home'}:
            return value
        def invoke(*args, **kwargs):
            return self._runtime._backend_call(name, value, *args, **kwargs)
        return invoke


class Runtime:
    supports_control_policy = True
    def __init__(self, ref, safety, backend_factory, *, device_id, environment, lock_root,
                 initial_holder='calibration', initialize_backend=None, jaw_mapping_file=None, data_plane=None, **owner_options):
        if environment not in {'fake', 'physical'} or initial_holder not in BUSINESSES:
            raise ValueError('explicit fake/physical environment and registered business required')
        if not isinstance(device_id, str) or not device_id.strip():
            raise ValueError('nonempty device identity required')
        hardware = environment == 'physical'
        if 'hardware' in owner_options and owner_options.pop('hardware') != hardware:
            raise ValueError('hardware flag differs from runtime namespace')
        if owner_options.get('position_environment',environment)!=environment:
            raise ValueError('position-boundary policy differs from runtime namespace')
        self.device_id, self.environment = device_id, environment
        self.data_plane=deepcopy(data_plane)
        self.ros_node=self.ros_publisher=None
        self.nonce, self.holder = secrets.token_hex(24), initial_holder
        self.lock, self.tick_lock = threading.RLock(), threading.Lock()
        self.tick_idle = threading.Condition(self.lock)
        self.clients, self.calls, self.hold = {}, {}, {'left': None, 'right': None}
        self.hold_returns = {'left': None, 'right': None}
        self.call_sequence, self.preview_revision = 0, 0
        self.calibration_revision, self.calibration_snapshot_id = 0, None
        self.reviewed_plans, self.execution_calibration = {}, None
        self.inference_factory, self.inference_configuration, self.inference_revision = None, None, 0
        self.last_return, self.stop_receipt, self.detached_hold = None, None, None
        self.release_receipt = None
        self.protected_retirement = None
        self.maintenance_active = False
        self.settings_write_pending = False
        self.control_operation = None
        self._hold_verification = None
        self.initialization_error = None
        self.owner = None
        root = Path(lock_root).resolve(); root.mkdir(parents=True, exist_ok=True)
        if owner_options.get('execution_root') is None:
            owner_options['execution_root']=root/'plan-evidence'
            owner_options['execution_root'].mkdir(mode=0o700,exist_ok=True)
        identity = hashlib.sha256((environment+'\0'+device_id).encode()).hexdigest()
        self.lock_path = root/(identity+'.lock')
        self.execution_settings_path = root/(identity+'.execution-settings.json')
        self.execution_settings_revision, self.execution_settings_policies = 0, {}
        self._policy_store_active = False
        self._policy_values = {}
        self._policy_origins = {}
        self._guard_policies = {}
        self._control_guards = {}
        self.fd = os.open(self.lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            if not stat.S_ISREG(os.fstat(self.fd).st_mode):
                raise ValueError('device lock must be a regular file')
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            from real_control import classes
            collection, _, _ = classes(ref, jaw_mapping_file=jaw_mapping_file,device_id=device_id)
            runtime = self
            class ManagedOwner(collection):
                def check_speed_protection(self, feedback):
                    # Read the committed policy at each sample. Full settings
                    # copies belong to snapshots and preparation, outside this check.
                    business = runtime.holder
                    policy = runtime._guard_policy(business)
                    speed_policy = policy['parameters']['measured']
                    observation=measured_joint_speed(feedback,speed_policy)
                    observation['guard_decision'] = Guard.evaluate(dict(rule_id='G14', signal_id=observation['joint'],
                        observed=observation['velocity_rad_s'], source_time=feedback['sample_time_s'],
                        checked_time=self.control_time(), epoch=self.epoch), 'execution', policy)
                    self.speed_protection_observation=observation
                    if (observation['guard_decision']['verdict']=='reject'
                            and self.control.state is not self.states.FAULT):
                        error=JointSpeedProtectionError(observation)
                        self.speed_protection_trip=dict(observation,business=runtime.holder,
                            revision=runtime.execution_settings_revision,command_epoch=self.epoch,reason=str(error),
                            trigger_qpos=feedback['qpos'][:],stop=dict(hold_return_confirmed=False,feedback_stationary_confirmed=False))
                        raise error
                    trip=getattr(self,'speed_protection_trip',None)
                    if trip and self.control.state is self.states.FAULT and self.control.reason.startswith(trip['reason']):
                        stop=trip['stop'];stop['current_feedback_peak_rad_s']=observation['peak_rad_s']
                        stop['maximum_joint_displacement_rad']=max(stop.get('maximum_joint_displacement_rad',0.),
                            max(abs(feedback['qpos'][i]-trip['trigger_qpos'][i])for i in(*range(6),*range(7,13))))
                        stopped=bool(stop['hold_return_confirmed']and self.feedback_healthy and self.stationary
                            and observation['peak_rad_s']<=observation['limit_rad_s'])
                        stop['currently_stationary']=stopped
                        if stopped and not stop['feedback_stationary_confirmed']:
                            stop.update(feedback_stationary_confirmed=True,stationary_sample_time_s=feedback['sample_time_s'],
                                elapsed_from_trigger_s=feedback['sample_time_s']-trip['sample_time_s'])
                            self.event('speed_protection_stop_observed',trip=deepcopy(trip))

                def recovery_ready(self):
                    observation=getattr(self,'speed_protection_observation',None)
                    below=observation is not None and (not observation['enabled'] or observation['peak_rad_s']<=observation['limit_rad_s'])
                    return (super().recovery_ready() or runtime._software_stop_recovery_ready()) and below

                def invalidate(self, *, normal_transition=False):
                    revoke=getattr(self.backend,'revoke_output',None)
                    if callable(revoke):revoke()
                    runner = getattr(self, 'runner', None)
                    if getattr(runner, 'is_inference', False):
                        runner.revoke('owner_invalidated',finish_data=not normal_transition)
                    getattr(self, 'inference_payloads', {}).clear()
                    getattr(self, 'reference_payloads', {}).clear()
                    return super().invalidate(normal_transition=normal_transition)

                def accept_review(self, plans):
                    # This short section is the execution/install ordering point;
                    # no SDK call, hashing, or trajectory compilation occurs here.
                    hashes = tuple(plan.sha256 for plan in plans)
                    with runtime.lock:
                        binding = runtime.reviewed_plans.pop(hashes, None)
                        if (binding is None or runtime.maintenance_active or runtime.settings_write_pending
                                or self.stop_requested or self.release_requested):
                            return False
                        guard=runtime._control_guards.get(binding['business'])
                        if guard is None:return False
                        decision=guard.evaluate(dict(rule_id='G01',signal_id=hashes[0],observed=runtime._review_identity(),
                            source_time=self.cache['calibration_feedback']['sample_time_s'],checked_time=self.control_time(),epoch=self.epoch),'admission')
                        self.event('guard_decision',**decision)
                        if decision['verdict']=='reject':return False
                        revoke = getattr(self.backend, 'revoke_output', None)
                        if callable(revoke):revoke()
                        self.execution_instance_id = secrets.token_hex(16)
                        self.execution_error_settings = (None if binding['execution_settings']['legacy_passive_guards']
                                                         else deepcopy(binding['execution_settings']['settings']))
                        runtime.execution_calibration = dict(binding, plan_sha256=hashes,
                            execution_instance_id=self.execution_instance_id,
                            accepted_monotonic_s=time.monotonic())
                        self.control_guard=runtime._control_guards[binding['business']]
                        return True

                def cached_snapshot_fields(self, result):
                    result = super().cached_snapshot_fields(result)
                    if not result.get('recoverable_fault') and runtime._software_stop_recovery_ready(result.get('control_protection')):
                        result['recoverable_fault']=self.control.reason
                        result['recovery_scope']='confirmed_software_stop_without_sdk_reconnect'
                    trip=getattr(self,'speed_protection_trip',None)
                    observation=getattr(self,'speed_protection_observation',None)
                    monitored=bool(result.get('connected') and result.get('feedback_healthy') and observation
                        and 0<=self.control_time()-observation['sample_time_s']<=.1)
                    result['speed_protection']=dict(runtime._execution_policy(runtime.holder)['speed_protection'],
                        business=runtime.holder,monitored=monitored,
                        observation=deepcopy(observation) if monitored else None,
                        last_trip=deepcopy(trip),latched=bool(trip and self.control.state is self.states.FAULT
                            and self.control.reason.startswith(trip['reason'])))
                    # Shared capabilities are invariant, including during the base cache refresh.
                    result['capabilities'] = ['collection_execution_v2',
                        'workbench_trajectory_v1', 'shared_manual_trajectory_v1', 'calibration_capture_ack_v1',
                        'calibration_capture_window_v2', 'single_arm_home', 'both_home_simultaneous', 'policy_inference_v1','research_control_v1']
                    result['inference'] = (self.runner.snapshot(receipt_view='none') if getattr(self.runner, 'is_inference', False) else None)
                    result['inference_configured'] = runtime.inference_factory is not None
                    result['inference_configuration_revision'] = runtime.inference_revision
                    interpreter=getattr(self.runner,'target_interpreter',None)
                    result['target_interpreter']=dict(enabled=bool(self.shared_target_profile),
                        controller_profile=self.shared_target_profile,
                        motion_gain_profile=getattr(self.backend,'motion_gain_profile',self.shared_target_profile),
                        native_reference_api=getattr(self.backend,'native_reference_api',None),
                        reference=interpreter.snapshot()if interpreter is not None else None)
                    return result

                def start_motion(self, plan, feedback, *, reviewed_deadline=None):
                    if runtime._prepare_control_capture(plan,reviewed_deadline):return
                    runtime.detached_hold = None
                    settings=(runtime.execution_calibration or {}).get('study_settings')
                    configure=getattr(self.backend,'configure_motion_gains',None)
                    if settings is not None and not callable(configure):raise ValueError('SDK adapter does not support study gains')
                    desired=None if settings is None else settings['gains']
                    integral=None if settings is None else settings['integral_enabled']
                    integral_gains=None if settings is None else settings.get('integral_gains')
                    if (desired!=getattr(self,'active_study_gains',None)or integral!=getattr(self,'active_study_integral',None)
                            or integral_gains!=getattr(self,'active_study_integral_gains',None)):
                        previous=getattr(getattr(self,'runner',None),'target_interpreter',None)
                        preserve=bool(desired is not None and getattr(self,'active_study_gains',None)is not None
                            and integral==getattr(self,'active_study_integral',None)
                            and integral_gains==getattr(self,'active_study_integral_gains',None)
                            and previous is not None and not previous.cancelled and previous.feedforward is None
                            and previous.epoch==self.epoch and previous.active_arm==plan.active_arm
                            and previous.profile==self.shared_target_profile and self.runner.status=='READY'
                            and self.control_time()>=previous.start_s+previous.reference.duration_s)
                        if preserve:
                            self.command_gate()
                            result=configure(desired,preserve_hold=True)
                            if result.get('preserved_hold')is not True:raise RuntimeError('gain change did not preserve hold')
                            self.event('study_gains_changed_in_hold',gains=desired,parent_reference_id=previous.reference.metadata['reference_id'])
                        else:
                            if previous is not None:previous.revoke()
                            if callable(configure):configure(desired)
                        self.active_study_gains=deepcopy(desired);self.active_study_integral=integral
                        self.active_study_integral_gains=deepcopy(integral_gains)
                    payload=getattr(self,'reference_payloads',{}).pop(plan.sha256,None)
                    if payload is not None:
                        reference,feedforward=payload
                        if (runtime.execution_calibration or {}).get('prepared_reference_id')!=reference.metadata['reference_id']:
                            raise ValueError('prepared reference review identity changed')
                        if (runtime.execution_calibration or {}).get('prepared_feedforward_sha256')!=(None if feedforward is None else feedforward['sha256']):
                            raise ValueError('prepared feedforward review identity changed')
                        self._bind_reference_context(velocity_source=reference.metadata.get('velocity_source'))
                        ref.Owner.start_motion(self,plan,feedback,reviewed_deadline=reviewed_deadline)
                        self.runner.prepared_reference=reference
                        if feedforward is not None:
                            factory=self.runner.interpreter_factory
                            def with_feedforward(current,side,stamp):
                                from .motion_reference import ReferenceFeedforward
                                interpreter=factory(current,side,stamp)
                                integral=interpreter.integrals.get('left')
                                interpreter.feedforward=ReferenceFeedforward(feedforward,stamp,current.duration_s,stamp,
                                    initial=None if integral is None else integral.bias.copy())
                                # The frozen profile already includes the prior total bias.
                                # Keep its inherited output only as the transition seed.
                                if integral is not None:
                                    integral.bias[:]=0.;integral.rate[:]=0.;integral.time=None
                                return interpreter
                            self.runner.interpreter_factory=with_feedforward
                        self.runner.reference_event=lambda result:self.event('control_study_native_window',**result)
                        self.runner.feedback_event=lambda now,state:self.event('control_study_feedback',source_time_s=now,raw=state,
                            reference_id=reference.metadata['reference_id'],reference_started_s=self.runner.started)
                        return
                    if plan.sha256 in self.inference_payloads:
                        payload = self.inference_payloads.pop(plan.sha256)
                        if runtime.inference_factory is None:
                            raise ValueError('inference backend is not prepared')
                        self.recording_error = None
                        self.arrival_pending = None
                        self._bind_reference_context(payload['configuration'].get('reference_planning'))
                        self.runner = runtime.inference_factory(self, plan, feedback, deepcopy(payload['configuration']))
                        self.runner.start(self.control_time(), feedback)
                        return
                    if plan.sha256 in self.frozen_payloads or plan.sha256 in self.alignment_plans:
                        self._bind_reference_context()
                        return super().start_motion(plan, feedback, reviewed_deadline=reviewed_deadline)
                    self._bind_reference_context()
                    return ref.Owner.start_motion(self, plan, feedback, reviewed_deadline=reviewed_deadline)

                def make_target_interpreter(self,reference,active_arm,start_s,**kwargs):
                    kwargs.setdefault('record_event',self.record_control_event)
                    settings=(runtime.execution_calibration or {}).get('study_settings')
                    if settings is not None:
                        kwargs['integral']=settings['integral_enabled']
                        kwargs['integral_gains']=settings.get('integral_gains')
                    return super().make_target_interpreter(reference,active_arm,start_s,**kwargs)

                def record_control_event(self,name,value):return runtime._capture_event(name,**value)

                def recording_failed(self,reason):
                    super().recording_failed(reason)
                    recording=runtime._control_capture()
                    if recording is None or getattr(self,'_recording_fault_run',None)==recording.execution_instance_id:return
                    self._recording_fault_run=recording.execution_instance_id
                    now=self.control_time()
                    phase='seal'if recording.capture._finishing or recording.done.is_set()else'recording'
                    decision=self.control_guard.evaluate(dict(rule_id='G38',signal_id=recording.execution_instance_id,
                        observed=1,source_time=now,checked_time=now,run_id=recording.execution_instance_id,epoch=self.epoch),phase)
                    self.event('guard_decision',**decision)
                    if decision['verdict']=='reject':
                        self._recording_stop_reason=str(reason)
                        ref.Owner.submit(self,dict(action='stop'))

                def stop(self,reason):
                    recording=runtime._control_capture()
                    detail=getattr(self,'_recording_stop_reason',None)
                    try:result=super().stop('required recording failed: '+detail if detail else reason)
                    except Exception as error:
                        if recording is not None:self._capture_finish_pending=(recording.execution_instance_id,'unknown','SDK Stop return unconfirmed: '+str(error))
                        raise
                    if recording is not None:self._capture_finish_pending=(recording.execution_instance_id,'failed'if detail else'stopped',str(detail or reason))
                    return result

                def tick(self):
                    result=super().tick()
                    runtime._advance_control_capture()
                    return result

                def reject_confirmation(self, plans, reason, *, review_deadline=None):
                    for plan in plans:
                        self.inference_payloads.pop(plan.sha256, None)
                    return super().reject_confirmation(plans, reason, review_deadline=review_deadline)

                def begin_collection(self, measured):
                    # Includes the physical collection implementation's direct gain calls.
                    return runtime._backend_call('begin_collection', super().begin_collection, measured)

            self.ref, self.collection = ref, collection
            backend = _Backend(backend_factory(), self)
            if getattr(backend,'target_interpreter_profile',None):
                # Warm CPU preparation libraries before the caller acquires SDK
                # output. Online fitting later stays in the existing query worker.
                from scipy.interpolate import CubicSpline
            self.owner = ManagedOwner(backend, safety, hardware=hardware, environment=environment, isolated_plans=True, **owner_options)
            self.owner.inference_payloads = {}
            self.owner.runtime_nonce = self.nonce
            self.owner.execution_clock_id='owner:'+self.nonce+':control_source'
            if data_plane is not None:
                from .ros_data import RosNode,SourceDirectory,plane_config
                from .ros_driver import RosDriverPublisher
                plane=plane_config(data_plane)
                if environment=='fake':
                    clock_id=getattr(backend,'source_clock_id',None)
                    if not isinstance(clock_id,str)or not clock_id:raise ValueError('ROS fake requires its original source clock identity')
                    self.owner.execution_clock_id=clock_id
                self.ros_node=RosNode('control_'+self.nonce[:12],namespace=plane['namespace'],
                    domain_id=plane['domain_id'],localhost_only=False)
                self.ros_directory=SourceDirectory(self.ros_node,device_id)
                self.ros_publisher=RosDriverPublisher(self.ros_node,device_id=device_id,instance_id=self.nonce,
                    clock_id=self.owner.execution_clock_id,state_topic=plane['namespace']+'/state',
                    event_topic=plane['namespace']+'/control/events')
            original_event=self.owner.event
            def captured_event(name,**fields):
                compact=fields
                if name not in ('feedback','command','source_target','native_reference_window'):
                    compact={key:value for key,value in fields.items()if key!='history'}
                    if 'history'in fields:compact['history_ref']='control.jsonl'
                    if isinstance(compact.get('motion_record'),dict):
                        compact['motion_record']={key:value for key,value in compact['motion_record'].items()if key not in('rows','native_samples')}
                        compact['motion_record']['rows_ref']='control.jsonl'
                if self.ros_publisher is not None:
                    fields=compact
                    from .inference_trace import clean
                    if isinstance(fields.get('raw'),dict) and 'qpos'in fields['raw']:
                        sample=fields['raw'].get('sample_time_s')
                        stamp=(fields['raw']['source_time_ns']if fields['raw'].get('source_clock_id')==self.ros_publisher.clock_id
                               else round(sample*1e9)if type(sample)in(int,float)and math.isfinite(sample)else None)
                        fields=dict(fields,raw=dict(state_ref=self.ros_publisher.reference('state',source_time_ns=stamp)if stamp is not None else None,
                            source_time_ns=stamp))
                    run_id=(fields.get('execution_instance_id')if name=='control_capture_sealed'
                            else getattr(self.owner,'execution_instance_id',''))or''
                    return self.ros_publisher.event(name,clean(fields),run_id=run_id,
                        epoch=self.owner.epoch,source_time_ns=round(self.owner.control_time()*1e9))
                if name not in ('feedback','command','source_target','native_reference_window'):
                    self._capture_event('owner_event',event=name,data=compact)
                return original_event(name,**fields)
            self.owner.event=captured_event
            if self.execution_settings_path.exists():
                self.restore_execution_settings(json.loads(self.execution_settings_path.read_text()))
            if initialize_backend is not None:
                # Caller owns connection/support authorization. No implicit connect/hold.
                with self.tick_lock:
                    try:
                        initialize_backend(backend)
                    except Exception as error:
                        # Retain ownership after a failed native initialization;
                        # never free the lock while compensation may still be live.
                        self.initialization_error = str(error)
                        self.owner.control.state = self.owner.states.FAULT
                        self.owner.control.reason = 'initialization failed: '+str(error)
        except BaseException:
            if self.ros_publisher is not None:self.ros_publisher.close()
            if self.ros_node is not None:self.ros_node.close()
            os.close(self.fd); self.fd = None
            raise

    def register_business(self, business):
        """Trusted server bootstrap only; never expose registration as a public route."""
        if business not in BUSINESSES:
            raise ValueError('unknown business')
        with self.lock:
            if business in self.clients:
                raise ValueError('business already registered')
            token = secrets.token_urlsafe(32)
            self.clients[business] = token
            return dict(business=business, token=token, runtime_nonce=self.nonce,
                        environment=self.environment, device_id=self.device_id)

    def control_inputs(self,*,include_feedback=True):
        """One cached control envelope for business preparation; never calls SDK."""
        if type(include_feedback)is not bool:raise ValueError('explicit feedback projection flag required')
        with self.lock,self.owner.lock:
            owner=self.owner
            feedback=owner.cache.get('calibration_feedback')
            publisher=getattr(self,'ros_publisher',None)
            reference=(publisher.reference('state',source_time_ns=feedback['source_time_ns']
                if feedback.get('source_clock_id')==publisher.clock_id else round(feedback['sample_time_s']*1e9))
                if publisher is not None and feedback else None)
            return dict(schema='workbench.control_inputs.v1',runtime_nonce=self.nonce,
                execution_clock_id=owner.execution_clock_id,
                command_epoch=owner.epoch,control_time_s=owner.control_time(),holder=self.holder,
                feedback_healthy=owner.feedback_healthy,
                calibration_feedback=deepcopy(owner.cache.get('calibration_feedback'))if include_feedback else None,
                **({'feedback_reference':reference}if publisher is not None else{}),
                joint_limits=deepcopy(owner.cache.get('joint_limits')),
                disabled_grippers=list(getattr(owner.backend,'passive_jaws',())),
                controller_profile=owner.shared_target_profile,
                native_reference_api=getattr(owner.backend,'native_reference_api',None),
                calibration_revision=self.calibration_revision,calibration_snapshot_id=self.calibration_snapshot_id,
                hardware_binding=str(owner.hardware_binding)if getattr(owner,'hardware_binding',None)else None)

    def revoke_inference(self):
        """Retire a business configuration atomically, without exporting Owner locks."""
        with self.lock,self.owner.lock:
            if self.holder!='inference':return
            owner=self.owner;runner=owner.runner;recording=owner.recording
            if (getattr(runner,'status',None)in('ARMED','PREPARING','VERIFYING','RUNNING','SETTLING')
                    or getattr(owner,'capture_sample',None)
                    or recording is not None and not recording.done.is_set() and not getattr(recording,'ingress_closed',False)):
                raise ValueError('设备执行或录制未结束，不能切换推理模型')
            self.preview_revision+=1
            owner.pending={token:review for token,review in owner.pending.items()
                if not any(plan.kind=='inference'for plan in review[0])}
            owner.inference_payloads.clear()
            self.reviewed_plans={key:value for key,value in self.reviewed_plans.items()if value.get('business')!='inference'}
            self.inference_factory=self.inference_configuration=None
            loop=getattr(runner,'loop',None)
            if loop is not None:
                loop.generation+=1;loop.history.clear();loop.applied.clear();loop._commands.clear();loop._outstanding=None
                runner.outstanding=None

    def configure_inference(self, configuration, factory):
        """Trusted prepared-model bootstrap; this never starts a model or moves a device."""
        from .inference_owner import configuration as validate, digest
        if not isinstance(configuration, dict):raise ValueError('inference configuration object required')
        if '_normal_finish'in configuration:raise ValueError('finish binding is resolved only by this Owner')
        if self._policy_store_active and 'inference' in self._policy_values:
            parameters = control_parameters(self._execution_policy('inference'), configuration=configuration, business='inference')
            self._overlay_policy_values('inference',parameters,configuration)
            value = self._configuration_from_policy(configuration, parameters)
        else:
            value = validate(configuration)
        if value.get('reference_planning') is not None:
            from .sdk_process import EXECUTION_CONTEXT_SCHEMA
            if getattr(self.owner.backend, 'execution_context_capabilities', None) != {'schema': EXECUTION_CONTEXT_SCHEMA}:
                raise ValueError('connected SDK worker lacks execution context capability')
        if value.get('normal_finish_policy')is not None:
            from .inference_owner import prepare_finish_binding
            value['_normal_finish']=prepare_finish_binding(value['normal_finish_policy'],self.owner,value['normal_finish_kind'])
        if value.get('action_chunk_binding')is not None:
            if value.get('action_chunk_clock_id',self.owner.execution_clock_id)!=self.owner.execution_clock_id:
                raise ValueError('action chunk clock differs from this Owner control source')
            value['action_chunk_clock_id']=self.owner.execution_clock_id
        if value.get('normal_modes')is not None:
            if value['normal_modes']['recording_required']and self.owner.listener is not None:
                raise ValueError('DAgger gesture ownership requires external intents; legacy Owner pedal toggling must be disabled')
            if (not self.owner.preserve_begin_jaws or not self.owner.shared_target_profile or getattr(self.owner.backend,'motion_gain_profile',self.owner.shared_target_profile)!='normal_X5'
                    or getattr(self.owner.backend,'native_reference_api',None)!='preserve_reference_v1'
                    or not all(callable(getattr(self.owner.backend,name,None))for name in('read_execution_output','set_execution_gains'))):
                raise ValueError('normal modes require preserved windows, normal_X5 and native output/gain capabilities')
        profile=self.owner.shared_target_profile
        if 'target_interpreter_profile'in value and value['target_interpreter_profile']!=profile:
            raise ValueError('inference reference controller profile differs from the shared device policy')
        if profile:
            value.update(target_interpreter_profile=profile,target_execution='shared_reference_windows')
            if value.get('reference_feedforward')and profile!='normal_X5':
                raise ValueError('frozen feedforward was qualified for normal_X5; requalify before another controller profile')
            if value.get('reference_feedforward')and getattr(self.owner.backend,'native_reference_api','legacy_recompute')!='legacy_recompute':
                raise ValueError('frozen inference feedforward requires native API requalification')
        if not callable(factory):
            raise ValueError('prepared inference runner factory required')
        with self._preview_boundary(), self.owner.lock:
            reason = self._idle_reason()
            if reason:
                raise ValueError(reason)
            parameters=control_parameters(self._execution_policy('inference'),configuration=value,business='inference')
            self._overlay_policy_values('inference',parameters,value)
            validate_control_parameters(parameters,environment=self.environment,business='inference')
            self.inference_configuration, self.inference_factory = value, factory
            self.inference_revision += 1
            self.preview_revision += 1
            self.owner.pending.clear()
            self.owner.inference_payloads.clear()
            self.reviewed_plans.clear()
            return dict(revision=self.inference_revision, configuration_digest=digest(value), configured=True)

    def export_execution_settings(self):
        with self.lock:
            return dict(schema='execution_settings.v1', device_id=self.device_id, environment=self.environment,
                revision=self.execution_settings_revision, policies=deepcopy(self.execution_settings_policies))

    def restore_execution_settings(self, value):
        """Trusted startup only; restore policies before this Runtime accepts clients."""
        with self.lock:
            if self.clients or self.owner.runner is not None:
                raise ValueError('execution settings restore is startup-only')
            if isinstance(value, dict) and value.get('schema') == 'workbench.control-policy-store.v1':
                required = {'schema', 'device_id', 'environment', 'revision', 'policies', 'origins'}
                if (set(value) != required or value['device_id'] != self.device_id or value['environment'] != self.environment
                        or type(value['revision']) is not int or value['revision'] < 0
                        or not isinstance(value['policies'], dict) or value['policies'].keys() - BUSINESSES
                        or not isinstance(value['origins'], dict)):
                    raise ValueError('control policy store identity or schema mismatch')
                parameters = {business: validate_control_parameters(policy, environment=self.environment,business=business)
                              for business, policy in value['policies'].items()}
                if any(business != 'inference' and (policy['execution'].keys() - {'fake_error'}
                        or any(group in policy for group in ('planning', 'model', 'observation')))
                       for business, policy in parameters.items()):
                    raise ValueError('inference policy fields require the inference business')
                policies = {business: execution_settings_from_policy(policy, environment=self.environment)
                            for business, policy in parameters.items()}
                self._configure_fake_error(policies.get(self.holder, default_execution_settings(self.holder)))
                self.execution_settings_revision = value['revision']
                self.execution_settings_policies = policies
                self._policy_values, self._policy_origins = parameters, deepcopy(value['origins'])
                for business,original in value['policies'].items():
                    if business=='inference':
                        for key in parameters[business]['observation']:
                            if key not in original.get('observation',{}):
                                self._policy_origins.setdefault(business,{})['observation.'+key]='migrated_input_defaults'
                self._policy_store_active = True
                self._guard_policies.clear()
                self._control_guards.clear()
                return
            if (not isinstance(value, dict) or value.get('schema') != 'execution_settings.v1'
                    or value.get('device_id') != self.device_id or value.get('environment') != self.environment
                    or type(value.get('revision')) is not int or value['revision'] < 0
                    or not isinstance(value.get('policies'), dict) or set(value['policies']) - BUSINESSES):
                raise ValueError('execution settings identity or schema mismatch')
            policies = {name: validate_execution_settings(policy, environment=self.environment,allow_legacy=True)
                        for name, policy in value['policies'].items()}
            self._configure_fake_error(policies.get(self.holder, default_execution_settings(self.holder)))
            self.execution_settings_revision, self.execution_settings_policies = value['revision'], policies
            self._guard_policies.clear()
            self._control_guards.clear()

    def _execution_policy(self, business):
        return deepcopy(self.execution_settings_policies.get(business) or default_execution_settings(business,
            self.owner.arm_tolerances or [self.owner.arm_tolerance] * 12))

    def execution_settings_snapshot(self, business):
        with self.lock:
            if business not in BUSINESSES:
                raise ValueError('unknown business')
            reason = ('请先交接控制权限：当前由 '+self.holder if business != self.holder else self._idle_reason(include_in_flight=False))
            planning_reason = ('请先交接推理控制权限' if business != 'inference' or business != self.holder
                               else self._planning_update_reason())
            return dict(revision=self.execution_settings_revision, business=business,
                settings=self._execution_policy(business), update_allowed=reason is None, update_reason=reason,
                planning_update_allowed=planning_reason is None, planning_update_reason=planning_reason,
                planning_limits=self._planning_limits()if business=='inference'else None,
                legacy_passive_guards=business == 'calibration' and business not in self.execution_settings_policies,
                fake_error_available=self.environment == 'fake' and callable(getattr(self.owner.backend, 'configure_error_model', None)))

    def _control_parameters(self, business):
        configuration = self.inference_configuration if business == 'inference' else None
        parameters = control_parameters(self._execution_policy(business), configuration=configuration, business=business)
        self._overlay_policy_values(business,parameters,configuration)
        return parameters

    def _planning_limits(self):
        outputs=(self.owner.last_begin_command or{}).get('source_outputs',{}).values()
        native=next((row.get('velocity_limits_sdk14')for row in outputs
                     if isinstance(row.get('velocity_limits_sdk14'),list)and len(row['velocity_limits_sdk14'])==14),None)
        limits={}
        if native is not None:
            from arx_env.joint_head_real_motion import jaw_peak_scale
            limits=dict(velocity_limit_rad_s=min(native[i]for i in (*range(6),*range(7,13))),
                jaw_velocity_limit_mm_s=min(native[slot]*jaw_peak_scale(side)for side,slot in (('left',6),('right',13))))
        protection=self._execution_policy('inference')['speed_protection']
        if protection['enabled']:
            limits['velocity_limit_rad_s']=min(limits.get('velocity_limit_rad_s',protection['limit_rad_s']),protection['limit_rad_s'])
        return limits

    def _overlay_policy_values(self,business,parameters,configuration):
        origins=self._policy_origins.get(business,{})
        for group,fields in self._policy_values.get(business,{}).items():
            for key,value in fields.items():
                if (business=='inference'and configuration is not None and group=='observation'and key=='maximum_age_s'
                        and origins.get('observation.maximum_age_s')=='product_default'):continue
                parameters.setdefault(group,{})[key]=deepcopy(value)

    def control_policy_snapshot(self, business):
        with self.lock:
            if business not in BUSINESSES:
                raise ValueError('unknown business')
            parameters = self._control_parameters(business)
            reason = ('请先交接控制权限：当前由 '+self.holder if business != self.holder else self._idle_reason(include_in_flight=False))
            origins = self._control_origins(business, parameters)
            return dict(revision=self.execution_settings_revision, policy_hash=policy_hash(parameters),
                        parameters=parameters, origins=deepcopy(origins), editable_paths=editable_policy_paths(parameters,business=business),
                        update_allowed=reason is None, update_reason=reason)

    def _guard_policy(self, business):
        # Bind once per revision, not once per 100 Hz feedback sample.
        key = (self.execution_settings_revision, self.inference_revision)
        cached = self._guard_policies.get(business)
        if cached is None or cached[0] != key:
            parameters = self._control_parameters(business)
            cached = (key, dict(revision=self.execution_settings_revision,
                                policy_hash=policy_hash(parameters), parameters=parameters))
            self._guard_policies[business] = cached
            self._control_guards[business]=Guard.bind(cached[1],self._guard_context())
        return cached[1]

    def _guard_context(self,limits=None):
        from .execution_settings import FEEDBACK_TIMEOUT_S,INTEGRAL_FEEDBACK_TIMEOUT_S,REFERENCE_STARVATION_MARGIN_S
        context=dict(environment=self.environment,feedback_timeout_s=FEEDBACK_TIMEOUT_S,
            integral_feedback_timeout_s=INTEGRAL_FEEDBACK_TIMEOUT_S,starvation_margin_s=REFERENCE_STARVATION_MARGIN_S)
        if limits is not None:
            from .motion_reference import JOINTS
            from arx_env.joint_head_jaw_mapping import command_gap_mm
            if len(limits)!=14:raise ValueError('bound SDK capability requires14 limits')
            context['joint_limits']=[list(limits[i])for i in JOINTS]
            context['jaw_limits_mm']=[sorted(command_gap_mm(side,float(value))for value in limits[slot])
                for side,slot in (('left',6),('right',13))]
            context['disabled_grippers']=list(getattr(self.owner.backend,'passive_jaws',()))
        return context

    def _review_identity(self):
        return dict(runtime_nonce=self.nonce,business=self.holder,command_epoch=self.owner.epoch,
            calibration_revision=self.calibration_revision,calibration_snapshot_id=self.calibration_snapshot_id,
            policy_revision=self.execution_settings_revision,inference_revision=self.inference_revision,
            controller_profile=self.owner.shared_target_profile,source_clock=deepcopy(self.owner.feedback_clock))

    def _control_origins(self, business, parameters):
        settings_source = 'persisted_execution_settings' if business in self.execution_settings_policies else 'runtime_defaults'
        origins = {}
        for path in editable_policy_paths(parameters):
            from_configuration = path.startswith(('planning.', 'observation.', 'model.', 'execution.prepare_drift_')) or path == 'execution.execute_steps'
            origins[path] = ('current_inference_binding' if self.inference_configuration else 'product_default') if from_configuration else settings_source
            if path.startswith('recording.'):
                origins[path] = 'approved_recording_contract' if path.endswith('purpose_profile') else 'recording_resource_defaults'
        origins.update(self._policy_origins.get(business, {}))
        return origins

    def _configuration_from_policy(self, value, parameters):
        if value is None:return None
        from .inference_owner import configuration
        result = deepcopy(value)
        result['execute_steps'] = parameters['execution'].get('execute_steps', result.get('execute_steps', 15))
        for key in ('prepare_drift_joint_rad', 'prepare_drift_jaw_mm'):
            if key in parameters['execution']:result[key] = parameters['execution'][key]
        if 'planning' in parameters:
            if result.get('reference_planning') is None:
                raise ValueError('selected model has no compatible planning policy binding')
            maximum=(result['reference_planning']['max_execution_duration_s']if result.get('finite_chunk_count')is not None
                else None)
            result['reference_planning'].update(parameters['planning'], max_execution_duration_s=maximum)
            if result.get('normal_modes'):
                result['normal_modes'].update(velocity_limit=parameters['planning']['velocity_limit_rad_s'],
                                              acceleration_limit=parameters['planning']['acceleration_limit_rad_s2'])
        if 'observation' in parameters:
            result['maximum_observation_age_s'] = parameters['observation']['maximum_age_s']
            for key in ('state_max_age_s','max_camera_span_s'):result[key]=parameters['observation'][key]
        if 'model' in parameters:result['reference_supply_timeout_s'] = parameters['model']['reference_supply_timeout_s']
        return configuration(result)

    def update_control_policy(self, credential, payload):
        if (not isinstance(payload, dict) or set(payload) != {'expected_revision', 'changes'}
                or type(payload['expected_revision']) is not int or payload['expected_revision'] < 0
                or not isinstance(payload['changes'], dict) or not payload['changes']):
            raise ValueError('control policy requires expected_revision and field changes')
        return self._update_policy(credential, changes=payload['changes'], expected_revision=payload['expected_revision'])

    def _configure_fake_error(self, settings):
        configure = getattr(self.owner.backend, 'configure_error_model', None)
        if callable(configure):
            configure(deepcopy(settings['fake_error']))
        elif settings['fake_error']['enabled']:
            raise ValueError('this backend does not support physical fake actuation errors')

    def update_execution_settings(self, credential, payload, *, expected_revision=None):
        settings = validate_execution_settings(payload, environment=self.environment)
        return self._update_policy(credential, settings=settings, expected_revision=expected_revision)

    def _update_policy(self, credential, *, settings=None, changes=None, expected_revision=None):
        planning_only = bool(changes) and set(changes) <= {'planning.velocity_limit_rad_s','planning.jaw_velocity_limit_mm_s'}
        with (self.lock if planning_only else self._preview_boundary()), self.owner.lock:
            business = self._authenticate(credential)
            future_steps = (business == 'inference' and changes is not None
                            and set(changes) == {'execution.execute_steps'})
            if business != self.holder and not future_steps:
                raise PermissionError('请先交接控制权限：当前由 '+self.holder+' 持有')
            reason = self._planning_update_reason() if planning_only and business == 'inference' else self._idle_reason()
            if reason:
                raise ValueError(reason)
            runner = self.owner.runner
            live_planning = bool(planning_only and business == 'inference' and getattr(runner,'normal_planned',False)
                                 and runner.status=='RUNNING' and not runner.normal_ended)
            if expected_revision is not None and expected_revision != self.execution_settings_revision:
                raise ValueError('control policy revision changed; read it again before committing')
            old = self._execution_policy(business)
            parameters = self._control_parameters(business)
            if changes is not None:
                parameters = merge_control_policy(parameters, changes, environment=self.environment,business=business)
                settings = execution_settings_from_policy(parameters, environment=self.environment)
            else:
                updated = control_parameters(settings, business=business)
                for group in ('measured', 'tracking', 'arrival'):
                    parameters[group] = updated[group]
                parameters['execution']['fake_error'] = updated['execution']['fake_error']
            parameters=validate_control_parameters(parameters,environment=self.environment,business=business)
            if planning_only and business=='inference':
                for name,maximum in self._planning_limits().items():
                    if parameters['planning'][name]>maximum:
                        raise ValueError(f'规划限速 {name} 超过当前设备／保护上限 {maximum:g}')
            configuration = self._configuration_from_policy(self.inference_configuration, parameters) if business == 'inference' else self.inference_configuration
            if business == self.holder and not live_planning:
                self._configure_fake_error(settings)  # Memory-only backend configuration; never sends a target.
            value = self.export_execution_settings()
            value['revision'] += 1
            value['policies'][business] = settings
            active = self._policy_store_active or changes is not None
            if active:
                policy_values = {name: self._control_parameters(name) for name in self.execution_settings_policies}
                policy_values.update(deepcopy(self._policy_values))
                policy_values[business] = parameters
                origins = {name: self._control_origins(name, policy) for name, policy in policy_values.items()}
                edited = changes.keys() if changes is not None else [path for path in editable_policy_paths(parameters)
                                                                     if path not in ('execution.execute_steps',) and not path.startswith(('planning.', 'observation.', 'model.', 'execution.prepare_drift_', 'recording.'))]
                for path in edited:
                    origins[business][path] = 'control_policy_api' if changes is not None else 'legacy_execution_settings_api'
                value = dict(schema='workbench.control-policy-store.v1', device_id=self.device_id, environment=self.environment,
                             revision=value['revision'], policies=policy_values, origins=origins)
            temporary = self.execution_settings_path.with_suffix('.tmp')
            self.settings_write_pending = True
        # Reserve new motion, not the control locks: slow storage must never
        # prevent feedback/heartbeat or Stop admission while the robot holds.
        try:
            temporary.write_text(json.dumps(value, sort_keys=True, allow_nan=False)+'\n')
            os.replace(temporary, self.execution_settings_path)
        except BaseException:
            with self.lock, self.owner.lock:
                self.settings_write_pending = False
                if business == self.holder and not live_planning:self._configure_fake_error(old)
            raise
        with self.lock, self.owner.lock:
            self.settings_write_pending = False
            self.execution_settings_revision = value['revision']
            if active:
                self._policy_store_active = True
                self._policy_values, self._policy_origins = value['policies'], value['origins']
                self.execution_settings_policies = {name: execution_settings_from_policy(policy, environment=self.environment)
                                                   for name, policy in self._policy_values.items()}
            else:
                self.execution_settings_policies = value['policies']
            if business == 'inference':self.inference_configuration = configuration
            if live_planning:
                if self.owner.runner is runner and not runner.normal_ended:
                    runner.pending_planning_update = (deepcopy(configuration['reference_planning']),
                                                      self.control_policy_snapshot('inference'))
                return self.control_policy_snapshot(business)
            if business == self.holder:
                self.owner.execution_error_settings = deepcopy(settings)
                # The completed native reference still owns the physical HOLD.
                # Retire reviews through their policy revision, preserving that
                # reference for the next Run's verified continuation. Stop keeps
                # its independent epoch revocation, including during disk I/O.
                self.owner.pending.clear()
            self.preview_revision += 1
            self.reviewed_plans.clear()
            self.owner.frozen_payloads.clear(); self.owner.alignment_plans.clear()
            self.owner.inference_payloads.clear()
            getattr(self.owner,'reference_payloads',{}).clear()
            return self.control_policy_snapshot(business) if changes is not None else self.execution_settings_snapshot(business)

    def _planning_update_reason(self):
        runner = self.owner.runner
        if not getattr(runner, 'normal_planned', False) or runner.normal_ended or runner.status!='RUNNING':
            return self._idle_reason(include_in_flight=False)
        if self.settings_write_pending or self.maintenance_active:
            return '配置保存或维护正在进行'
        owner = self.owner
        if not owner.backend.is_connected or not owner.feedback_healthy:
            return '需要已连接且反馈健康的控制会话'
        if owner.stop_requested or owner.release_requested or not owner.commands.empty():
            return '控制命令正在处理'
        if runner.normal_single_segment or runner.mode not in ('MODEL', 'HOLD', 'TEACH') or runner.normal_job:
            return '请等待当前模式切换完成'
        if runner.status in ('STOPPED', 'FAULT'):
            return '当前推理控制会话已停止'
        return None

    def _authenticate(self, credential):
        if self.fd is None:
            raise PermissionError('runtime closed; reconnect through the device lifecycle')
        if not isinstance(credential, dict):
            raise PermissionError('registered server identity required')
        business = credential.get('business')
        expected = self.clients.get(business)
        token = credential.get('token')
        if (not expected or not isinstance(token, str) or not secrets.compare_digest(token, expected)
                or credential.get('runtime_nonce') != self.nonce
                or credential.get('environment') != self.environment or credential.get('device_id') != self.device_id):
            raise PermissionError('business credential belongs to another runtime/device/environment')
        return business

    def _control_capture(self):
        owner=self.owner
        recording=getattr(owner,'recording',None)
        return recording if (getattr(recording,'recording_backend',None)=='capture_session_v1'
            and recording.execution_instance_id==getattr(owner,'execution_instance_id',None)) else None

    def _capture_event(self,kind,**fields):
        if self.ros_publisher is not None:
            from .inference_trace import clean
            if kind=='feedback' and fields.get('phase')=='sdk_read':
                runner = getattr(self.owner, 'runner', None)
                classify = getattr(runner, 'memory_sampling_activity', None)
                activity = (classify(fields['raw']['sample_time_s'], fields['sample_epoch'])
                            if callable(classify) else
                            dict(schema='workbench.memory-activity.v2', run_id=None, phase='HOLD'))
                self.ros_publisher.publish_state(dict(fields['raw'],source_call_id=fields.get('call_id'),
                    runtime_nonce=self.nonce,command_epoch=fields['sample_epoch'],memory_sampling=activity))
                return
            self.ros_publisher.event(kind,clean(fields),run_id=getattr(self.owner,'execution_instance_id','')or'',
                epoch=self.owner.epoch,source_time_ns=round(self.owner.control_time()*1e9))
            return
        recording=self._control_capture()
        if recording is None or not recording.ready.is_set()or recording.capture._finishing:return
        try:recording.submit(kind,**fields)
        except Exception as error:self.owner.recording_failed(error)

    def _prepare_control_capture(self,plan,reviewed_deadline):
        owner=self.owner;factory=getattr(owner,'recording_factory',None)
        binding=self.execution_calibration or{}
        if (binding.get('execution_instance_id')!=getattr(owner,'execution_instance_id',None)or not binding
                or plan.sha256 in getattr(owner,'inference_payloads',{})or getattr(factory,'recording_backend',None)!='capture_session_v1'):return False
        recording=self._control_capture()
        if recording is None:
            from .execution_settings import CAPTURE_PREPARATION_TIMEOUT_S
            from dataclasses import asdict
            identity=owner.execution_instance_id;epoch=owner.epoch;business=binding['business']
            active=lambda:(owner.execution_instance_id==identity and owner.epoch==epoch and self.holder==business
                and not owner.stop_requested and not owner.release_requested)
            frozen=getattr(owner,'frozen_payloads',{}).get(plan.sha256)
            execution_context=dict(device_id=self.device_id,environment=self.environment,
                operator_intent=binding.get('operator_intent'),
                calibration_snapshot_id=binding.get('snapshot_id'),calibration_revision=binding.get('calibration_revision'),
                controller_profile=binding.get('controller_profile'),study_settings=deepcopy(binding.get('study_settings')),
                source_clock=deepcopy(binding.get('inference_clock')),jaw_mapping=deepcopy(owner.cache.get('jaw_mapping')),
                native_reference_api=getattr(owner.backend,'native_reference_api',None))
            payload=dict(frozen[0] if frozen is not None else {},execution_owner=dict(owner_pid=os.getpid(),runtime_nonce=self.nonce,command_epoch=epoch),
                recording_preparation=dict(execution_instance_id=identity,business_kind=business,control_policy=binding['control_policy'],
                    execution_context=execution_context,
                    preparation_active=active,preparation_deadline_monotonic=time.monotonic()+CAPTURE_PREPARATION_TIMEOUT_S,defer_execution=True))
            recording=factory(payload,owner.recording_failed)
            def sealed(value):
                callback=getattr(self,'capture_result_handler',None)
                if callback is not None:callback(value)
                owner.event('control_capture_sealed',**value)
            recording.on_sealed=sealed
            if getattr(recording,'recording_backend',None)!='capture_session_v1':raise ValueError('common control recorder required')
            owner.recording=recording;owner.recording_error=None;owner._recording_fault_run=None;owner._recording_stop_reason=None
            recording.capture.config['planned_motion']=asdict(plan)
        if recording.error:owner.recording_failed(recording.error)
        if recording.ready.is_set()and not recording.error:return False
        from arx_env.joint_head_real_motion import MotionRunner
        previous=owner.runner
        owner.runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=owner.motion_fault,
            consumed=owner.consumed,environment=self.environment,execution_settings=getattr(owner,'execution_error_settings',None),
            guard=owner.control_guard,disabled_grippers=getattr(owner.backend,'passive_jaws',()))
        self._capture_pending=dict(plan=plan,previous=previous,placeholder=owner.runner,epoch=owner.epoch,
            review_age=owner.review_age(),reviewed_deadline=reviewed_deadline)
        return True

    def _advance_control_capture(self):
        owner=self.owner;recording=self._control_capture()
        if recording is None:return
        if not recording.capture._finishing:
            error=recording.error
            if error:owner.recording_failed(error)
        pending=getattr(self,'_capture_pending',None)
        if pending is not None:
            if (owner.epoch!=pending['epoch']or owner.runner is not pending['placeholder']
                    or owner.control.state is owner.states.FAULT or owner.stop_requested or owner.release_requested):
                self._capture_pending=None
            elif recording.error or (not recording.prepared.is_set()and time.monotonic()>=recording.preparation_deadline_monotonic):
                owner.recording_failed(recording.error or 'common recording preparation deadline exceeded')
            elif recording.ready.is_set():
                self._capture_pending=None;owner.runner=pending['previous'];owner.accepted_review_age_s=pending['review_age']
                try:owner.start_motion(pending['plan'],owner.cache['calibration_feedback'],reviewed_deadline=pending['reviewed_deadline'])
                except Exception as error:owner.motion_fault(error)
                finally:owner.accepted_review_age_s=None
        ending=getattr(owner,'_capture_finish_pending',None)
        if ending is not None:
            owner._capture_finish_pending=None
            identity,status,reason=ending
            if identity==recording.execution_instance_id:
                if status=='stopped'and owner.control.state is owner.states.FAULT:status='failed'
                recording.finish(status,reason)
        elif (getattr(self,'_capture_pending',None)is None and owner.runner is not None
                and owner.runner.status=='READY' and not owner.sequence_remaining and owner.capture_sample is None
                and not owner.capture_wait and not getattr(owner.runner,'recording_waits_for_endpoint',False)):
            recording.finish('completed')
        elif not recording.capture._finishing and owner.control.state in (owner.states.FAULT,owner.states.RELEASED):
            recording.finish('unknown'if getattr(owner.backend,'terminal_failure',None)or owner.control.state is owner.states.RELEASED else'failed',
                owner.control.reason)

    def _backend_call(self, name, function, *args, **kwargs):
        entered,entry_cpu=time.monotonic(),time.thread_time_ns()
        with self.lock:
            acquired,acquired_cpu=time.monotonic(),time.thread_time_ns()
            if name in {'begin', 'send', 'begin_collection', 'enable_collection_arm','queue_reference','set_execution_gains','bind_execution_context','start_native_home'} or name=='configure_motion_gains'and kwargs.get('preserve_hold'):
                self.owner.command_gate()
            self.call_sequence += 1
            call = dict(id=self.call_sequence, operation=name, started_monotonic_s=time.monotonic(),
                        entered_monotonic_s=entered,admission_lock_acquired_s=acquired,
                        admission_lock_wait_s=acquired-entered,admission_lock_cpu_s=(acquired_cpu-entry_cpu)/1e9,
                        epoch=self.owner.epoch if self.owner else None)
            call['command_gate_and_registration_s']=call['started_monotonic_s']-acquired
            if self._hold_verification is not None and name in ('begin','send'):
                call.update(side=args[0],**{('feedback_argument' if name=='begin' else 'target'):list(args[1])})
            if self._hold_verification is not None and name=='queue_reference':
                from .sdk_process import ReferenceWindowRejected
                packet=args[0];target=self.control_operation['fixed_target']
                if (not packet['q_sdk14'] or any(list(row)!=target for row in packet['q_sdk14'])
                        or any(value!=0 for field in ('dq_sdk14','torque_bias_sdk14') for row in packet[field] for value in row)):
                    raise ReferenceWindowRejected('fixed hold requires unchanged q and zero dq/added torque')
                call.update(target=list(target),reference_id=packet['reference_id'],
                    node_count=len(packet['q_sdk14']),zero_velocity=True,zero_added_torque=True)
            self.calls[call['id']] = call
        call_cpu=time.thread_time_ns();backend_started=time.monotonic();result=None;failure_timing=None
        try:
            try:
                if name in CONTROL_OUTPUTS:
                    self._capture_event('command',operation=name,phase='attempt',call_id=call['id'],arguments=args,options=kwargs)
                    if self._control_capture() is not None and name not in ('set_damping_hold','close'):
                        self.owner.command_gate()
                result = function(*args, **kwargs)
            finally:
                backend_returned,backend_cpu=time.monotonic(),time.thread_time_ns()
        except BaseException as error:
            self._capture_event('owner_event',event='sdk_call_failed',operation=name,call_id=call['id'],error=str(error))
            if name=='queue_reference':failure_timing=getattr(error,'timing',None)
            with self.lock:
                call['error'] = str(error)
                if name == 'close':
                    if self.release_receipt is None:
                        self.release_receipt = dict(runtime_nonce=self.nonce, sdk_close_returned=False)
                    self.release_receipt.update(status='unconfirmed', message='SDK 释放失败：'+str(error))
                    self.owner.release_requested = False  # A failed explicit release needs a new operator request.
                if name in {'begin', 'send', 'begin_collection', 'set_damping_hold', 'close'}:
                    self.hold = dict(left=None, right=None)
                    self.hold_returns = dict(left=None, right=None)
            raise
        else:
            if name in('read_calibration_state','read_execution_output'):
                self._capture_event('feedback',phase='sdk_read'if name=='read_calibration_state'else'native_output_read',
                    raw=result,call_id=call['id'],sample_epoch=call['epoch'])
            elif name in CONTROL_OUTPUTS:self._capture_event('command',operation=name,phase='returned',call_id=call['id'],result=result)
            with self.lock:
                if name == 'set_damping_hold':
                    self.hold = dict.fromkeys(('left', 'right'), 'manual_damping' if kwargs.get('manual') else 'damping')
                    self.hold_returns = dict.fromkeys(('left', 'right'), call)
                elif name == 'begin':
                    if self._hold_verification is not None:call['returned_feedback']=list(result)
                    self.hold[args[0]] = 'position'
                    self.hold_returns[args[0]] = call
                elif name == 'begin_collection':
                    self.hold = dict.fromkeys(('left', 'right'), 'position')
                    self.hold_returns = dict.fromkeys(('left', 'right'), call)
                elif name == 'close':
                    self.hold = dict(left=None, right=None)
                    self.hold_returns = dict(left=None, right=None)
                    if self.release_receipt is not None:
                        self.release_receipt['sdk_close_returned'] = True
            return result
        finally:
            with self.lock:
                call['returned_monotonic_s'] = time.monotonic()
                call['cpu_s']=(time.thread_time_ns()-call_cpu)/1e9
                call.update(backend_started_s=backend_started,backend_returned_s=backend_returned,
                    backend_wall_s=backend_returned-backend_started,backend_cpu_s=(backend_cpu-call_cpu)/1e9,
                    return_bookkeeping_s=call['returned_monotonic_s']-backend_returned,
                    total_wall_s=call['returned_monotonic_s']-entered)
                elapsed=call['total_wall_s']
                from .pipeline_timing import timing
                timing.record('sdk.'+name,round(entered*1e9),ended_ns=round(call['returned_monotonic_s']*1e9),
                    epoch=call.get('epoch'),runtime_nonce=self.nonce,call_id=call['id'],
                    backend_ns=round(call['backend_wall_s']*1e9),
                    admission_wait_ns=round(call.get('admission_lock_wait_s',0)*1e9))
                if self.owner is not None and (elapsed>=.01 or failure_timing is not None)and name not in('connect','close'):
                    details={k:result[k]for k in('feedback_read_timing','sdk_worker_read_timing','sdk_client_read_timing','sdk_reference_timing')if isinstance(result,dict)and k in result}
                    if failure_timing is not None:details['sdk_reference_timing']=failure_timing
                    self.owner.event('sdk_call_latency',call=deepcopy(call),timing=deepcopy(details),
                        runtime_nonce=self.nonce,business=self.holder,threshold_s=.01)
                if name=='close' and self.release_receipt is not None:
                    backend=getattr(function,'__self__',None)
                    self.release_receipt['release_diagnostics']=dict(schema='sdk_release_diagnostics.v1',
                        owner_call={key:call[key] for key in ('id','operation','started_monotonic_s','returned_monotonic_s','error') if key in call},
                        owner_steps=deepcopy(list(getattr(backend,'release_steps',()))),
                        can_evidence='unsupported' if self.environment=='fake' else 'independent_guard',
                        scope='Owner memory is available only if native close returned or raised; guard records survive protected termination')
                self.last_return = call
                if self._hold_verification is not None and name in ('begin','send','queue_reference','set_damping_hold'):
                    self.control_operation['sdk_returns'].append(deepcopy(call))
                self.calls.pop(call['id'])
                self.tick_idle.notify_all()

    @contextmanager
    def _preview_boundary(self):
        with self.lock:
            started=time.monotonic();sequence=self.call_sequence;before=deepcopy(list(self.calls.values()))
            # Waiting releases the runtime lock: stop and the current SDK return stay independent.
            if not self.tick_idle.wait_for(lambda:not self.tick_lock.locked() and not self.calls, timeout=.1):
                self.owner.event('preview_boundary_timeout',waited_s=time.monotonic()-started,
                    call_sequence_before=sequence,call_sequence_after=self.call_sequence,
                    in_flight_before=before,in_flight_after=deepcopy(list(self.calls.values())),
                    tick_locked=self.tick_lock.locked(),last_sdk_return=deepcopy(self.last_return))
                raise ValueError('control tick or SDK call is still in flight after 0.1s preview boundary wait')
            yield

    def _release_reason(self, *, include_in_flight=True):
        owner = self.owner
        if self.settings_write_pending:return '执行配置正在保存；停止仍可独立使用'
        if include_in_flight and (self.tick_lock.locked() or self.calls):
            return '控制调用尚未结束，暂不能释放连接'
        if not owner.backend.is_connected:
            return '设备已断开，等待连接收尾'
        if owner.release_requested:
            return '释放已受理，等待 SDK 断开确认'
        if (owner.runner is not None and owner.runner.status not in {'READY','STOPPED','FAULT'}
                or owner.sequence_remaining or owner.capture_sample):
            return '请先停止运动和采图，再释放设备连接'
        if getattr(owner.runner, 'recording_pending', False):
            return '推理录制尚未完成收尾，暂不能释放连接'
        recording = owner.recording
        if recording is not None and (not recording.done.is_set() or getattr(recording,'thread',None) is None or recording.thread.is_alive()):
            return '录制尚未完成收尾，暂不能释放连接'
        return None

    def request_release(self, credential, command=None, *, maintenance=False, actor=None):
        command = {'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'} if command is None else command
        if command != {'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'}:
            raise ValueError('explicit supported-arm release required')
        with self._preview_boundary():
            if actor is not None:
                actor.require(self)
                if actor.action!='release_control':raise PermissionError('maintenance actor cannot release this device')
                business=self.holder;maintenance=True
            else:business = self._authenticate(credential)
            if self.maintenance_active and not maintenance:
                raise ValueError('服务维护中，释放需要维护身份')
            if business != self.holder:
                raise PermissionError('请先交接控制权限：当前由 '+self.holder+' 持有')
            reason = self._release_reason()
            if reason:
                raise ValueError(reason)
            self.preview_revision += 1
            method = self.ref.Owner.submit if business == 'calibration' else self.collection.submit
            result = method(self.owner, command)
            if getattr(self.owner.runner, 'is_inference', False):
                self.owner.runner.revoke('operator release')
            self.release_receipt = dict(status='requested', runtime_nonce=self.nonce,
                calibration_snapshot_id=self.calibration_snapshot_id,
                requested_monotonic_s=time.monotonic(), requested_wall_ns=time.time_ns(),
                sdk_close_returned=False, message='释放已受理，等待 SDK 断开与设备锁收尾')
            if self.owner.last_fault_diagnostic is not None:
                self.release_receipt['fault_diagnostic']=deepcopy(self.owner.last_fault_diagnostic)
            return dict(result, release_receipt=deepcopy(self.release_receipt))

    def _idle_reason(self, *, include_in_flight=True, maintenance_actor=None):
        if self.settings_write_pending:return '执行配置正在保存；停止仍可独立使用'
        if self.maintenance_active:
            if maintenance_actor is None:return '服务维护中，停止仍可独立使用'
            maintenance_actor.require(self)
        owner = self.owner
        if include_in_flight and (self.tick_lock.locked() or self.calls):
            return 'control tick or SDK call is still in flight'
        if not owner.backend.is_connected:
            return 'device disconnected'
        if owner.stop_requested or owner.release_requested or not owner.commands.empty():
            return 'control command still pending'
        if (owner.runner is not None and owner.runner.status not in {'READY', 'STOPPED'}
                or owner.sequence_remaining or owner.capture_sample):
            return 'motion or capture is still active'
        if getattr(owner.runner, 'recording_pending', False) and not getattr(owner.recording,'ingress_closed',False):
            return 'inference recording ingress is not closed'
        if owner.control.state not in (owner.states.READY, owner.states.STOPPED, owner.states.MANUAL):
            return 'holding/control state is not confirmed'
        if any(mode is None for mode in self.hold.values()):
            return 'no successful SDK holding-mode return evidence'
        feedback = owner.cache.get('calibration_feedback') or {}
        if (not owner.feedback_healthy or not owner.stationary
                or not 0 <= owner.control_time()-feedback.get('sample_time_s', -1) <= .1):
            return 'fresh healthy stationary feedback required'
        if owner.pedal_snapshot()[1:] != (False, True):
            return 'pedal must be released and healthy'
        recording = owner.recording
        if recording is not None and not getattr(recording,'ingress_closed',False):
            if not recording.done.is_set() or getattr(recording, 'thread', None) is None or recording.thread.is_alive():
                return 'recording writer has not exited after sealing'
        return None

    def begin_maintenance(self):
        """Claim only between control calls; queued confirmations cannot cross."""
        with self._preview_boundary(), self.owner.lock:
            if self.maintenance_active:raise ValueError('已有服务维护窗口')
            reason=self._release_reason()
            output_exited=False
            if reason and getattr(self.owner.backend,'terminal_failure',None):
                evidence=self.owner.backend.protection_snapshot()
                recording=self.owner.recording
                if (evidence.get('worker_exit_confirmed') is True and not self.tick_lock.locked() and not self.calls
                        and (recording is None or recording.done.is_set()
                             and getattr(recording,'thread',None) is not None and not recording.thread.is_alive())):
                    reason=None;output_exited=True  # Inspect/retire only; old SDK output cannot resume.
            if reason:raise ValueError(reason)
            if not output_exited and (self.owner.stop_requested or not self.owner.commands.empty()):
                raise ValueError('控制命令仍在排队，等待收尾后再维护')
            self.maintenance_active=True
            self.preview_revision+=1
            self.owner.pending.clear();self.owner.inference_payloads.clear();self.reviewed_plans.clear()

    def end_maintenance(self):
        with self.lock:self.maintenance_active=False

    def request_stop(self):
        """Same independent stop path for business and restricted control actors."""
        with self.lock:
            if self.fd is None:return dict(message='设备连接已释放，没有新的控制输出')
            result=self.ref.Owner.submit(self.owner,{'action':'stop'})
            if getattr(self.owner.runner,'is_inference',False):self.owner.runner.revoke('operator stop')
            self.preview_revision+=1
            self.stop_receipt=dict(accepted_monotonic_s=time.monotonic(),barrier_epoch=self.owner.epoch,
                in_flight_calls=deepcopy(list(self.calls.values())),prior_calls_complete=not self.calls,
                stop_hold_return_confirmed=False,firmware_ack='unsupported',physical_stop='unverified')
            recording=self.owner.recording
            if recording is not None and getattr(recording,'recording_backend',None)!='capture_session_v1':
                def finish():
                    try:recording.finish('stopped','operator stop')
                    except Exception as error:self.owner.recording_failed(error)
                threading.Thread(target=finish,daemon=True).start()
            return dict(result,stop=deepcopy(self.stop_receipt))

    def control_release(self,actor):return self.request_release(None,actor=actor)

    def _software_stop_recovery_ready(self, protection=None):
        """Recognize only an evidenced Stop/reference race; never mutate on read."""
        owner=self.owner;receipt=self.stop_receipt or {};diagnostic=owner.last_fault_diagnostic or {}
        timing=diagnostic.get('target_translation_timing') or {};barrier=receipt.get('barrier_epoch')
        if (getattr(owner,'native_faults',{}) or owner.control.state is not owner.states.FAULT or owner.control.reason not in
                ('target reference epoch revoked','target reference revoked during native admission')
                or diagnostic.get('error')!=owner.control.reason or type(barrier)is not int
                or diagnostic.get('command_epoch')!=barrier or type(timing.get('epoch'))is not int
                or not timing['epoch']<barrier<=owner.epoch
                or timing.get('error_type')not in ('ValueError','ReferenceRevokedError')):return False
        accepted=receipt.get('accepted_monotonic_s');returned=timing.get('returned_s')
        if not all(type(x)in(int,float)and math.isfinite(x)for x in(accepted,returned))or not 0<=accepted<=returned:return False
        prior=receipt.get('in_flight_calls')
        if not isinstance(prior,list)or any(not isinstance(row,dict)or type(row.get('id'))is not int for row in prior):return False
        calls=self.calls.copy()
        if (self._release_reason(include_in_flight=False) is not None or owner.stop_requested
                or owner.release_requested or not owner.commands.empty()
                or any(row['id']in calls for row in prior)
                or any(row['operation']not in {'read_calibration_state','get_bimanual_joint_position_limits','read_gains'}
                       for row in calls.values())
                or getattr(owner.backend,'terminal_failure',None)
                or not owner.feedback_healthy or not owner.stationary or owner.pedal_snapshot()[1:]!=(False,True)):
            return False
        feedback=owner.cache.get('calibration_feedback')or{}
        if not owner.recovery_feedback_valid(feedback,owner.cache.get('joint_limits')):return False
        if not 0<=owner.control_time()-feedback.get('sample_time_s',-1)<=.1:return False
        if self.environment=='physical':
            guard=feedback.get('can_output_guard')or{};heartbeat=guard.get('heartbeat_monotonic_s')
            if (guard.get('state')!='armed'or type(heartbeat)not in(int,float)
                    or not 0<=owner.control_time()-heartbeat<=.1):return False
        if set(self.hold_returns)!={'left','right'}or set(self.hold)!={'left','right'}:return False
        if not all(self.hold[side]=='damping' and row and row.get('operation')=='set_damping_hold'
                and 'error'not in row and row.get('epoch',-1)>=barrier
                and row.get('started_monotonic_s',-1)>=accepted
                and row.get('returned_monotonic_s',-1)>=row['started_monotonic_s']
                for side,row in self.hold_returns.items()):return False
        if self.environment=='physical' and feedback['sample_time_s']<max(
                row['returned_monotonic_s'] for row in self.hold_returns.values()):return False
        # The Owner tick rechecks its fresh armed Guard feedback above. Do not
        # add a status-file read to this control-thread recovery gate.
        if protection is None:return True
        return bool(protection and protection.get('active_anomaly')is False
            and protection.get('recovery_required')is False and protection.get('event')is None
            and not protection.get('worker_exit_confirmed')and not protection.get('supervisor_exit_confirmed')
            and (protection.get('heartbeat')or{}).get('status')=='healthy'
            and (protection.get('heartbeat')or{}).get('monitored')is True)

    def control_recover(self,actor):
        actor.require(self)
        if actor.action!='recover_control':raise PermissionError('maintenance actor cannot recover this device')
        with self._preview_boundary(),self.owner.lock:
            actor.require(self)
            if not self.owner.recovery_ready():raise ValueError('当前不是可确认恢复的运动故障；不会自动清除设备或guard锁存')
            result=self.ref.Owner.submit(self.owner,dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',owner_pid=os.getpid(),command_epoch=self.owner.epoch))
            self.preview_revision+=1
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            actor.require(self)
            facts=self.fact_snapshot()
            from .state_contract import control_healthy
            if (self.owner.recoverable_fault is None and self.owner.control.state is self.owner.states.STOPPED
                    and control_healthy(facts)):
                return dict(result,recovered=True,runtime=facts,motion_resumed=False)
            time.sleep(.01)
        raise ValueError('恢复请求结果未确认；未重连或恢复旧动作')

    def verify_hold(self,actor):
        """Fixed five-second joint-pose diagnostic with existing jaw command bounds."""
        actor.require(self)
        if actor.action!='verify_hold':raise PermissionError('maintenance actor cannot run fixed hold verification')
        with self._preview_boundary(),self.owner.lock:
            actor.require(self)
            reason=self._idle_reason(maintenance_actor=actor)
            if reason:raise ValueError(reason)
            if self._hold_verification is not None:raise ValueError('固定保持验证尚未结束')
            self.owner.invalidate();self.owner.runner=None;self.preview_revision+=1
            self.control_operation=dict(schema='workbench.fixed_hold_verification.v1',operation_id=secrets.token_hex(16),
                status='queued',verified=False,duration_s=5.,runtime_nonce=self.nonce,command_epoch=self.owner.epoch,
                command_transport='native_window' if self.owner.shared_target_profile else 'point_send',
                disabled_grippers=list(getattr(self.owner.backend,'passive_jaws',())),target_source='fresh_joint_feedback_and_bounded_calibrated_jaw_command',
                joint_offsets_applied=False,requested_target=None,fixed_target=None,samples=[],sdk_returns=[],
                stop_receipt=None,motion_kind='fixed_joint_pose_hold_not_home',firmware_ack='unsupported')
            verification=dict(actor=actor,done=threading.Event(),runner=None,epoch=self.owner.epoch,stop_sent=False,error=None,
                initial_stop_receipt=self.stop_receipt,
                guard=Guard.bind(self._guard_policy(self.holder),self._guard_context(self.owner.cache['joint_limits'])))
            self._hold_verification=verification
        if not verification['done'].wait(30):
            self.request_stop()
            with self.lock:self.control_operation.update(status='unknown',error='固定保持/Stop回执超时，停止已请求但结果未确认')
            raise ValueError(self.control_operation['error'])
        result=deepcopy(self.control_operation)
        if result['status']!='completed':raise ValueError(result.get('error') or '固定保持验证已撤销，部分证据保留在环境状态')
        return result

    def _hold_before_tick(self):
        verification=self._hold_verification
        if verification is None or verification['stop_sent']:return
        owner=self.owner;actor=verification['actor']
        try:
            actor.require(self)
            if owner.epoch!=verification['epoch']:raise ValueError('保持验证的控制代次已变化')
            if verification['runner'] is not None:return
            reason=self._idle_reason(include_in_flight=False,maintenance_actor=actor)
            if reason:raise ValueError(reason)
            from arx_env.joint_head_real_motion import MotionRunner,make_plan,feedback_to_command
            feedback=deepcopy(owner.cache['calibration_feedback']);limits=deepcopy(owner.cache['joint_limits'])
            owner.control_guard=verification['guard']
            def plan(raw):return make_plan('both',raw,feedback_to_command(raw,limits,environment=self.environment),limits,
                duration_s=5.,kind='workbench',created_at_s=owner.control_time(),environment=self.environment)
            def begin(measured):
                actor.require(self);actual=owner.begin('both',measured)
                # The native begin re-samples the current pose. Seal that exact
                # raw pose once so interpolation cannot move toward an older pose.
                runner.plan=plan(actual)
                runner.consumed.add(runner.plan.sha256)
                from arx_env.joint_head_jaw_mapping import feedback_gaps,command_gap_mm,command_max_mm
                measured_gaps=feedback_gaps(actual)
                command_gaps=[command_gap_mm(side,runner.plan.q_target[index]) for side,index in (('left',6),('right',13))]
                self.control_operation.update(jaw_feedback_mm=measured_gaps,jaw_command_mm=command_gaps,
                    jaw_command_max_mm=[command_max_mm(side)for side in ('left','right')],
                    jaw_command_clipped={side:abs(measured_gaps[i]-command_gaps[i])>1e-6 for i,side in enumerate(('left','right'))})
                self.control_operation.update(status='holding',begin_feedback_qpos=list(actual),fixed_target=list(runner.plan.q_target),
                    started_wall_ns=time.time_ns(),started_monotonic_s=time.monotonic(),error_policy=deepcopy(runner.error_settings))
                return actual
            def send(desired):
                actor.require(self)
                if tuple(desired)!=tuple(self.control_operation['fixed_target']):raise ValueError('固定保持目标发生变化')
                owner.send('both',desired)
            runner=MotionRunner(plan(feedback['qpos']),begin=begin,send=send,stop=owner.motion_fault,consumed=owner.consumed,
                arm_tolerance=owner.arm_tolerance,arm_tolerances=owner.arm_tolerances,environment=self.environment,
                execution_settings=owner.motion_error_settings(),disabled_grippers=getattr(owner.backend,'passive_jaws',()),
                guard=verification['guard'],
                interpreter_factory=(lambda reference,side,stamp:owner.make_target_interpreter(reference,side,stamp,integral=False))
                    if owner.shared_target_profile else None)
            owner.active_epoch=owner.epoch;owner.review_press=owner.pedal_snapshot()[0];owner.runner=runner
            verification['runner']=runner
            self.control_operation.update(status='verifying_feedback',initial_feedback=feedback)
            runner.start(owner.control_time(),feedback)
        except Exception as error:
            verification['error']=str(error);verification['stop_sent']=True
            if self.stop_receipt is verification['initial_stop_receipt']:self.request_stop()

    def _hold_after_tick(self):
        verification=self._hold_verification
        if verification is None:return
        owner=self.owner;receipt=self.control_operation;runner=verification['runner']
        feedback=owner.cache.get('calibration_feedback')
        if feedback is not None:
            receipt['samples'].append({key:deepcopy(feedback[key]) for key in ('qpos','qvel','sample_time_s','sample_wall_ns','source_time_ns','physics_step','sdk_timestamps') if key in feedback})
        if not verification['stop_sent']:
            if not verification['actor'].current() or owner.epoch!=verification['epoch'] or runner is None or runner.status in ('FAULT','STOPPED'):
                verification['error']=getattr(runner,'reason',None) or 'Stop、维护范围或设备条件已变化'
                verification['stop_sent']=True
                if self.stop_receipt is verification['initial_stop_receipt']:self.request_stop()
            elif runner.started is not None and owner.control_time()-runner.started>=5.:
                receipt.update(status='stopping',held_source_s=owner.control_time()-runner.started,
                    held_wall_s=time.monotonic()-receipt['started_monotonic_s'],target_error=deepcopy(runner.target_error))
                verification['stop_sent']=True;self.request_stop()
        if verification['stop_sent'] and (self.stop_receipt or {}).get('stop_hold_return_confirmed') is True:
            self._finish_hold_verification(verification['error'])

    def _finish_hold_verification(self,error):
        verification=self._hold_verification;receipt=self.control_operation
        receipt.update(status='failed' if error else 'completed',verified=error is None,error=error,
            stop_receipt=deepcopy(self.stop_receipt),finished_wall_ns=time.time_ns(),sample_count=len(receipt['samples']),
            sdk_send_count=sum(row['operation']=='send' and 'error' not in row for row in receipt['sdk_returns']),
            sdk_window_count=sum(row['operation']=='queue_reference' and 'error' not in row for row in receipt['sdk_returns']))
        self._hold_verification=None;verification['done'].set()

    def submit(self, credential, command):
        if not isinstance(command, dict):
            raise ValueError('command object required')
        action = command.get('action')
        if action=='preview_frozen':
            from .control_plan import RawPreview,ArtifactPreview,validate_python_input
            if not isinstance(command,(RawPreview,ArtifactPreview)):validate_python_input(command)
        study=None
        if 'study_settings'in command:
            if action!='preview_workbench' or self.owner.shared_target_profile!='x5_control50_damping_v1':
                raise ValueError('study settings require the declared V10 research route')
            command=dict(command);study=validate_study(command.pop('study_settings'))
        if action == 'release':
            return self.request_release(credential, command)
        with self._preview_boundary() if action in CALIBRATION_PREVIEWS | COLLECTION_PREVIEWS | INFERENCE_PREVIEWS else self.lock:
            business = self._authenticate(credential)
            if study is not None and business!='research':raise PermissionError('study settings require research control ownership')
            if action == 'stop':
                if command!={'action':'stop'}:raise ValueError('explicit stop command required')
                return self.request_stop()
            if self.settings_write_pending:raise ValueError('执行配置正在保存；停止仍可独立使用')
            if self.maintenance_active:raise ValueError('服务维护中，停止仍可独立使用')
            if business != self.holder:
                raise PermissionError('请先交接控制权限：当前由 '+self.holder+' 持有')
            if action=='normal_mode':
                if business!='inference'or not getattr(self.owner.runner,'is_inference',False):
                    raise ValueError('normal mode requires the active inference Owner')
                return self.owner.runner.submit_mode(command)
            if action=='normal_finish':
                if business!='inference'or not getattr(self.owner.runner,'is_inference',False):
                    raise ValueError('normal finish requires the ending inference Owner')
                return self.owner.runner.submit_finish(command)
            previews = {'calibration': CALIBRATION_PREVIEWS, 'collection': COLLECTION_PREVIEWS,
                        'inference': INFERENCE_PREVIEWS,'research':set()}[business] | {'preview_workbench'}
            if action in previews:
                previous=self.owner.runner
                if (action=='preview_inference'and getattr(previous,'normal_ended',False)
                        and previous.config.get('_normal_finish')and not previous.finish_completed):
                    raise ValueError('the previous episode requires completed normal finish before the next review')
                if action=='preview_workbench' and business!='calibration' and isinstance(command.get('waypoints'),list) and any(
                        isinstance(row,dict) and row.get('capture') for row in command.get('waypoints',[])):
                    raise ValueError('capture acknowledgement belongs to calibration')
                reason = self._idle_reason()
                if reason:
                    raise ValueError(reason)
                self.preview_revision += 1
                revision, epoch = self.preview_revision, self.owner.epoch
                pedal = self.owner.pedal_snapshot()
                with self.owner.lock:
                    # A compiler snapshot has no control loop and never touches SDK.
                    compiler = copy(self.owner)
                    compiler.lock = threading.RLock()
                    compiler.cache = deepcopy(self.owner.cache)
                    compiler._preview_live_owner = self.owner
                    compiler._preview_current = lambda:(self.preview_revision==revision and self.holder==business
                        and self.owner.epoch==epoch and not self.maintenance_active)
                    compiler.pending, compiler.frozen_payloads, compiler.alignment_plans = {}, {}, {}
                    compiler.inference_payloads = {}
                    compiler.reference_payloads = {}
                    compiler.commands = queue.Queue(maxsize=16)
                    compiler.execution_error_settings = (None if business == 'calibration' and business not in self.execution_settings_policies
                                                         else self._execution_policy(business))
                policy_parameters=self._control_parameters(business)
                if (getattr(getattr(self.owner,'recording_factory',None),'recording_backend',None)=='capture_session_v1'
                        and business!='inference' and policy_parameters['recording']['purpose_profile'] is None):
                    raise ValueError('research recording purpose must be selected before review')
                policy_revision=self.execution_settings_revision
                review_identity=self._review_identity()
                if action=='preview_workbench' and 'sampled_trajectory' in command:
                    if study is not None:raise ValueError('Raw replay cannot select research gains')
                    from .raw_replay import preview as replay_preview
                    method=lambda current,requested:replay_preview(self.ref,current,requested,
                        device_id=self.device_id,calibration_snapshot_id=self.calibration_snapshot_id,
                        maximum_velocity=self._execution_policy(business)['speed_protection'])
                elif action=='preview_workbench' and 'reference_artifact' in command:
                    from .control_reference import preview,start_allowance_matches
                    from .target_interpreter import gains
                    method=lambda current,requested:preview(self.ref,current,requested,
                        allow_j3_integral_pause=bool(business=='research'and study and study['integral_enabled']),
                        allow_start_allowance=bool(business=='research'and start_allowance_matches(current,study,command)))
                elif action == 'preview_inference':
                    if self.inference_factory is None:
                        raise ValueError('prepare the inference model before review')
                    from .inference_owner import preview as inference_preview
                    config = deepcopy(self.inference_configuration)
                    config.update(calibration_snapshot_id=self.calibration_snapshot_id,
                                  calibration_revision=self.calibration_revision)
                    method = lambda current, requested: inference_preview(current, requested, config)
                else:
                    method = self.ref.Owner.submit if business == 'calibration' or action in ('preview_workbench','preview_operator_home','preview_operator_hold') else self.collection.submit
            else:
                if action not in {'confirm', 'capture_ack', 'recover', 'release', 'simulate_pedal'}:
                    raise ValueError('action unavailable for this business')
                if action == 'capture_ack' and business != 'calibration':
                    raise ValueError('capture acknowledgement belongs to calibration')
                if action in {'confirm', 'release', 'recover', 'simulate_pedal'}:
                    self.preview_revision += 1
                method = self.ref.Owner.submit if business == 'calibration' else self.collection.submit
                result = method(self.owner, command)
                if action == 'simulate_pedal' and getattr(self.owner.runner, 'is_inference', False):
                    self.owner.runner.revoke('operator pedal')
                return result
        # No runtime/owner lock around full plan validation and hashing.
        frozen_policy=dict(revision=policy_revision,policy_hash=policy_hash(policy_parameters),parameters=policy_parameters)
        context=self._guard_context(compiler.cache['joint_limits']);context['expected_identity']=review_identity
        if action=='preview_inference':
            from .inference_owner import episode_duration
            context['run_duration_s']=episode_duration(command.get('duration_s',config['duration_s']))
            mapping=config.get('conversion_identity',{}).get('mapping',{})
            model_bounds=mapping.get('command_jaw_limits_mm')
            if model_bounds is not None:
                context['model_jaw_limits_mm']=[[max(low,model_bounds[0]),min(high,model_bounds[1])]
                    for low,high in context['jaw_limits_mm']]
            else:
                maximum=mapping.get('profile_max_opening_mm')
                if config.get('binding',{}).get('schema')=='workbench.wam-native-coordinates.v1':
                    if not isinstance(maximum,dict)or set(maximum)!={'left','right'}:
                        raise ValueError('WAM per-side calibrated jaw ranges required')
                    context['model_jaw_limits_mm']=[[low,min(high,maximum[side])]
                        for side,(low,high)in zip(('left','right'),context['jaw_limits_mm'])]
                else:
                    context['model_jaw_limits_mm']=[[low,high if maximum is None else min(high,maximum)]for low,high in context['jaw_limits_mm']]
            if config.get('normal_modes') is not None:
                context.update(normal_brake_s=config['normal_modes']['brake_duration_s'],gain_transition_s=config['normal_modes']['gain_transition_s'])
        frozen_guard=Guard.bind(frozen_policy,context)
        from .joint_bounds import JointPositionBoundaryError
        try:result = method(compiler, command)
        except JointPositionBoundaryError as error:
            detail=error.details
            guard=Guard.bind(frozen_policy,dict(environment=self.environment,
                joint_limits=[[detail['lower_rad'],detail['upper_rad']]]))
            source=compiler.cache['calibration_feedback']['sample_time_s']
            decision=guard.evaluate(dict(rule_id='G20'if detail['context']=='feedback'else'G19',
                signal_id=detail['joint'],observed=[detail['value_rad']],source_time=source,
                checked_time=self.owner.control_time(),epoch=epoch),'preparation')
            error.details['guard_decision']=decision
            compiler.event('guard_decision',**decision)
            raise
        from .motion_reference import JOINTS
        for plans,*_ in compiler.pending.values():
            for plan in plans:
                decision=frozen_guard.evaluate(dict(rule_id='G19',signal_id=plan.sha256,
                    observed=[plan.requested_q_target[i]for i in JOINTS],
                    source_time=compiler.cache['calibration_feedback']['sample_time_s'],
                    checked_time=self.owner.control_time(),epoch=epoch),'preparation')
                compiler.event('guard_decision',**decision)
                if decision['verdict']=='reject':raise ValueError('G19: prepared command exceeds current device projection allowance')
        if study is not None and 'integral_gains'in study:
            for reference,feedforward in compiler.reference_payloads.values():
                if feedforward is not None and feedforward['provenance'].get('study_settings')!=study:
                    raise ValueError('研究前馈的完整积分增益配置与本次审阅不一致')
        with self._preview_boundary(), self.owner.lock:
            if (self.holder != business or self.owner.epoch != epoch or self.preview_revision != revision
                    or self.execution_settings_revision!=policy_revision
                    or self._idle_reason() or self.owner.cache.get('joint_limits') != compiler.cache.get('joint_limits')
                    or self.owner.pedal_snapshot() != pedal):
                raise ValueError('preview revoked during compilation; request a new preview')
            self.owner.pending = compiler.pending
            self.owner.frozen_payloads = compiler.frozen_payloads
            self.owner.alignment_plans = compiler.alignment_plans
            self.owner.inference_payloads = compiler.inference_payloads
            self.owner.reference_payloads = compiler.reference_payloads
            self.owner.review_wall_deadline = compiler.review_wall_deadline
            self._guard_policies[business]=((self.execution_settings_revision,self.inference_revision),frozen_policy)
            self._control_guards[business]=frozen_guard
            self.reviewed_plans = {
                tuple(plan.sha256 for plan in review[0]): dict(prepared_reference_id=(compiler.reference_payloads[review[0][0].sha256][0].metadata['reference_id'] if review[0][0].sha256 in compiler.reference_payloads else None),
                    prepared_feedforward_sha256=(compiler.reference_payloads[review[0][0].sha256][1]['sha256'] if review[0][0].sha256 in compiler.reference_payloads and compiler.reference_payloads[review[0][0].sha256][1] is not None else None),calibration_revision=self.calibration_revision,
                    inference_revision=self.inference_revision,
                    control_policy=frozen_policy,
                    controller_profile=self.owner.shared_target_profile,operator_intent=action if action in ('preview_operator_home','preview_operator_hold')else None,
                    study_settings=deepcopy(study),
                    inference_clock=deepcopy(self.owner.feedback_clock),
                    snapshot_id=self.calibration_snapshot_id, command_epoch=epoch, business=business,
                    execution_settings=dict(revision=self.execution_settings_revision, business=business,
                                            legacy_passive_guards=business == 'calibration' and business not in self.execution_settings_policies,
                                            settings=self._execution_policy(business)))
                for review in compiler.pending.values()}
            return dict(result, business=business, runtime_nonce=self.nonce, environment=self.environment,
                target_interpreter=dict(enabled=bool(self.owner.shared_target_profile),
                    controller_profile=self.owner.shared_target_profile,ordinary_point_send_disabled=bool(self.owner.shared_target_profile)))

    def notify_calibration_installed(self, snapshot_id):
        """Trusted installer callback after publishing the actual installed identity.

        Running/accepted sequences retain their frozen payload and command epoch.
        Pending reviews, queued confirms and compilers lose only their approval.
        """
        if not isinstance(snapshot_id, str) or not snapshot_id.strip() or len(snapshot_id) > 512:
            raise ValueError('actual installed calibration snapshot identity required')
        with self.lock, self.owner.lock:
            if self.fd is None:
                raise RuntimeError('runtime closed')
            if snapshot_id == self.calibration_snapshot_id:
                return dict(snapshot_id=snapshot_id, calibration_revision=self.calibration_revision, revoked=False)
            self.calibration_snapshot_id = snapshot_id
            self.calibration_revision += 1
            self.preview_revision += 1
            self.owner.pending.clear()
            self.reviewed_plans.clear()
            # Do not clear frozen_payloads: a review accepted immediately before
            # this callback may not yet have constructed its collection runner.
            return dict(snapshot_id=snapshot_id, calibration_revision=self.calibration_revision, revoked=True)

    def handoff(self, credential, target, *, active=None):
        with self.lock:
            business=self._authenticate(credential)
            if business!=self.holder or target==business:raise PermissionError('current holder must explicitly hand off to another registered business')
        return self.admit_business(target,active=active,expected_holder=business)

    def admit_business(self,target,*,active=None,expected_holder=None,maintenance_actor=None):
        """Shared trusted admission, preserving the existing Owner boundary/hold."""
        with self._preview_boundary():
            if active is not None and not active():raise ValueError('handoff revoked before control boundary')
            if target not in self.clients or expected_holder is not None and expected_holder!=self.holder:
                raise PermissionError('registered business and unchanged current holder required')
            if maintenance_actor is not None:
                maintenance_actor.require(self)
                if maintenance_actor.action!='ops_prepare_collection' or target!='collection':
                    raise PermissionError('maintenance actor cannot claim this business')
            reason = self._idle_reason(maintenance_actor=maintenance_actor)
            if reason:
                raise ValueError(reason)
            if target==self.holder:return dict(holder=target,command_epoch=self.owner.epoch,reused=True)
            if not self.tick_lock.acquire(blocking=False):
                raise ValueError('control tick is in flight')
            try:
                with self.owner.lock:
                    self._configure_fake_error(self._execution_policy(target))
                    self.owner.invalidate()
                    self.preview_revision += 1
                    self.owner.frozen_payloads.clear(); self.owner.alignment_plans.clear()
                    self.owner.inference_payloads.clear()
                    # READY runners only monitor endpoint hold. Retire their old
                    # policy/epoch; the SDK retains its already submitted setpoint.
                    self.owner.runner = None
                    self.owner.active_epoch = None
                    self.detached_hold = dict(modes=deepcopy(self.hold), epoch=self.owner.epoch,
                                              sdk_returns=deepcopy(self.hold_returns),
                                              evidence='completed SDK calls; firmware ACK unsupported')
                    self.holder = target
                    self.owner.execution_error_settings = (None if target == 'calibration' and target not in self.execution_settings_policies
                                                           else self._execution_policy(target))
                    self.owner.cache.update(command_epoch=self.owner.epoch, motion=None, automatic_enabled=False)
                    self.owner.cache.pop('frozen', None)
                    return dict(holder=target, command_epoch=self.owner.epoch, hold=deepcopy(self.detached_hold))
            finally:
                self.tick_lock.release()

    def tick(self):
        from .pipeline_timing import timing
        began=time.monotonic_ns();previous=getattr(self,'_timing_tick_ns',None);self._timing_tick_ns=began
        if previous is not None:timing.record('control.tick_interval',previous,ended_ns=began)
        try:return self._tick()
        finally:timing.record('control.tick',began)

    def _tick(self):
        with self.lock:
            if not self.tick_lock.acquire(blocking=False):
                raise RuntimeError('only one control tick may run at a time')
        try:
            if self.fd is None:
                raise RuntimeError('runtime closed')
            protection_failure=getattr(self.owner.backend,'terminal_failure',None)
            if protection_failure:
                from .gc_timing import snapshot as gc_snapshot
                timing_fault=gc_snapshot()
                with self.lock,self.owner.lock:
                    self.owner.invalidate()
                    self.owner.control.state=self.owner.states.FAULT
                    self.owner.control.auto_aborted=True
                    self.owner.control.reason=str(protection_failure)
                    if self.owner.runner:
                        self.owner.runner.status,self.owner.runner.reason='FAULT',str(protection_failure)
                    self.owner.cache.update(state='FAULT_STOP_UNCONFIRMED',reason=str(protection_failure),
                        connected=False,automatic_enabled=False,feedback_healthy=False,stationary=False,
                        command_epoch=self.owner.epoch,control_timing_fault=timing_fault)
                    self.owner.stop_requested=self.owner.release_requested=False
                    while not self.owner.commands.empty():self.owner.commands.get_nowait()
                    self.hold=dict(left=None,right=None);self.hold_returns=dict(left=None,right=None)
                self.owner.recording_call('finish','failed',str(protection_failure))
                self.owner.event('control_protection_timing',reason=str(protection_failure),gc=timing_fault)
                if self._hold_verification is not None:self._finish_hold_verification(str(protection_failure))
                return False
            self._hold_before_tick()
            result = self.owner.tick()
            with self.lock:
                if self.stop_receipt is not None:
                    receipt = self.stop_receipt
                    receipt['prior_calls_complete']=all(row['id'] not in self.calls for row in receipt['in_flight_calls'])
                    stopped = self.owner.control.state in (self.owner.states.STOPPED, self.owner.states.MANUAL, self.owner.states.FAULT)
                    if (not receipt['stop_hold_return_confirmed'] and stopped and not self.calls and all(row is not None
                            and row['operation'] == 'set_damping_hold' and 'error' not in row
                            and row.get('epoch',-1)==receipt['barrier_epoch']+1
                            and row.get('started_monotonic_s', -1) >= receipt['accepted_monotonic_s']
                            and 'returned_monotonic_s' in row for row in self.hold_returns.values())):
                        receipt.update(stop_hold_return_confirmed=True, sdk_hold_returns=deepcopy(self.hold_returns))
                self._hold_after_tick()
            return result
        finally:
            with self.lock:
                self.tick_lock.release()
                self.tick_idle.notify_all()

    def snapshot(self, credential):
        with self.lock:return self._snapshot(self._authenticate(credential))

    def fact_snapshot(self):
        """Shared cached device facts; no business registration or impersonation."""
        with self.lock:return self._snapshot(None)

    def _snapshot(self, business):
        with self.lock:
            reason = '请先交接控制权限：当前由 '+self.holder+' 持有' if business is not None and business != self.holder else self._idle_reason(include_in_flight=False)
            readiness_reason = self._idle_reason(include_in_flight=False)
            # Use the cache: a stalled SDK/recording call must not block status.
            with self.owner.lock:
                state = deepcopy(self.owner.cache)
                publication=state.get('control_publication') or {}
                published_epoch=(publication.get('dependencies') or {}).get('command_epoch')
                coherent=(publication.get('valid') is True and published_epoch==self.owner.epoch
                    and state.get('state')==self.owner.control.state.value)
                coherence=dict(consistent=coherent,published_epoch=published_epoch,
                    current_epoch=self.owner.epoch,publication_revision=publication.get('revision'))
            owner=self.owner
            protection_reader=getattr(owner.backend,'protection_snapshot',None)
            protection=protection_reader() if callable(protection_reader) else None
            if protection is not None:
                state['control_protection']=protection
                from .gc_timing import snapshot as gc_snapshot
                state['control_timing']=dict(gc=gc_snapshot(),last_sdk_return=deepcopy(self.last_return))
                if protection.get('event'):
                    state.update(connected=protection.get('transport_connected') is True,state='FAULT_STOP_UNCONFIRMED',automatic_enabled=False,
                        feedback_healthy=False,stationary=False,reason=protection['reason'])
            state=owner.cached_snapshot_fields(state)
            native=getattr(owner,'native_task_status',None)
            state['native_task']=({key:deepcopy(native[key]) for key in (
                'execution_instance_id','execution_timeline_digest','source_clock_id','task_outcome','task_qualified',
                'receipt_path','receipt_sha256','runtime_nonce','physics_step','message','first_failure') if key in native}
                if isinstance(native,dict) else None)
            recording=owner.recording
            if recording is None:
                state['recording']=None
            else:
                manifest=getattr(recording,'manifest',None)
                root=getattr(recording,'root',None)
                active=getattr(recording,'thread',None)
                state['recording']=dict(ready=recording.ready.is_set(),done=recording.done.is_set()and not(active and active.is_alive()),
                    ingress_closed=getattr(recording,'ingress_closed',False),
                    error=recording.error,controller_result=deepcopy(getattr(recording,'controller_result',None)),
                    diagnostics_error=getattr(recording,'diagnostics_error',None),seal_error=getattr(recording,'seal_error',None),
                    directory=str(root) if root else None,
                    manifest_path=str(root/'manifest.json') if root and manifest else None,
                    manifest={k:deepcopy(v) for k,v in manifest.items() if k!='files'} if manifest else None)
            state.update(device_id=self.device_id, environment=self.environment, runtime_nonce=self.nonce,
                control_time_s=owner.control_time(),
                control_coherence=coherence,
                maintenance_active=self.maintenance_active,
                holder=self.holder, command_epoch=self.owner.epoch, hold=deepcopy(self.hold),
                calibration_revision=self.calibration_revision, calibration_snapshot_id=self.calibration_snapshot_id,
                execution_calibration=deepcopy(self.execution_calibration),
                execution_settings=self.execution_settings_snapshot(self.holder if business is None else business),
                execution_instance_id=getattr(self.owner,'execution_instance_id',None),
                sdk_hold_returns=deepcopy(self.hold_returns),
                sdk_calls_in_flight=deepcopy(list(self.calls.values())), last_sdk_return=deepcopy(self.last_return),
                detached_hold=deepcopy(self.detached_hold), stop_receipt=deepcopy(self.stop_receipt),
                release_receipt=deepcopy(self.release_receipt),
                initialization_error=self.initialization_error,
                prior_control_retirement=deepcopy(getattr(self,'prior_control_retirement',None)),
                control_operation=({**{key:deepcopy(value) for key,value in self.control_operation.items()
                    if key not in ('samples','sdk_returns')},
                    'sample_count':len(self.control_operation.get('samples',[])),
                    'sdk_return_count':len(self.control_operation.get('sdk_returns',[])),
                    'full_evidence':'operation_receipt'} if self.control_operation else None),
                idle_readiness=dict(allowed=readiness_reason is None,reason=readiness_reason,
                    control_cycle_in_flight=bool(self.tick_lock.locked() or self.calls)),
                allowed_operations={'stop':dict(allowed=True, reason=None),
                    'release':dict(allowed=business == self.holder and self._release_reason(include_in_flight=False) is None,
                        reason='请先交接控制权限：当前由 '+self.holder+' 持有' if business != self.holder else self._release_reason(include_in_flight=False)),
                    'preview':dict(allowed=reason is None, reason=reason),
                    'handoff':dict(allowed=reason is None, reason=reason),
                    'confirm':dict(allowed=reason is None and bool(self.owner.pending),
                                   reason=reason or (None if self.owner.pending else 'generate and review a fresh plan'))})
            if self.maintenance_active:
                for action,gate in state['allowed_operations'].items():
                    if action!='stop':gate.update(allowed=False,reason='服务维护中')
            if business is None:
                state.pop('allowed_operations',None)
                state['control_operations']=dict(stop=dict(allowed=True,reason=None),
                    release=dict(allowed=self._release_reason(include_in_flight=False) is None,reason=self._release_reason(include_in_flight=False)),
                    recover=dict(allowed=self.owner.recovery_ready(),reason=None if self.owner.recovery_ready() else '当前没有满足恢复条件的运动故障'))
            return state

    def close(self):
        """Release the managed lock only after an explicitly authorized disconnect."""
        with self.lock:
            if self.tick_lock.locked() or self.calls or self.owner.backend.is_connected:
                raise RuntimeError('explicit supported release and completed SDK calls required before closing runtime')
            if self.release_receipt is not None and not self.release_receipt['sdk_close_returned']:
                raise RuntimeError('SDK release return remains unconfirmed')
            if self.fd is not None:
                listener=self.owner.listener
                if listener is not None and callable(getattr(listener,'close',None)):
                    listener.close()
                    self.owner.listener=None
                os.close(self.fd); self.fd = None
            if self.release_receipt is not None:
                self.release_receipt.update(status='released', confirmed_wall_ns=time.time_ns(),
                    message='设备连接已释放；再次接入需要显式连接')
        if self.ros_publisher is not None:self.ros_publisher.close()
        if self.ros_node is not None:self.ros_node.close()

    def retire_protected(self):
        """Explicit supported reconnect only; worker exit is never SDK close success."""
        with self.lock:
            backend=self.owner.backend
            reader=getattr(backend,'protection_snapshot',None)
            evidence=reader() if callable(reader) else {}
            recording=self.owner.recording
            if (not evidence.get('event') or evidence.get('worker_exit_confirmed') is not True
                    or self.tick_lock.locked() or self.calls or backend.is_connected
                    or recording is not None and not recording.done.is_set()):
                raise ValueError('保护后的控制进程退出或录制收尾尚未确认，不能重新接入')
            self.owner.invalidate()
            listener=self.owner.listener
            if listener is not None:listener.close();self.owner.listener=None
            if self.fd is not None:os.close(self.fd);self.fd=None
            backend.dispose()
            self.protected_retirement=dict(runtime_nonce=self.nonce,event=evidence['event'],
                retired_wall_ns=time.time_ns(),worker_exit_confirmed=True,sdk_close_confirmed=False,
                mechanical_state='not_verified',motion_resumed=False)
            result=deepcopy(self.protected_retirement)
        if self.ros_publisher is not None:self.ros_publisher.close()
        if self.ros_node is not None:self.ros_node.close()
        return result
