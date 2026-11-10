"""One inference business on the shared Device, cameras, Owner and RGB-D writer."""
from copy import deepcopy
from .execution_settings import DEFAULT_JOINT_TOLERANCE_RAD
import json
import os
import secrets
from pathlib import Path
import threading
import time
import weakref
from urllib.parse import urlencode

import numpy as np

from .inference import vector14
from .inference_model import (CAMS, LegacyBaseAdapter, ContinuousBaseAdapter, letterbox_rgb224,
                             model_family, model_spec, model_bundle_digest)
from .inference_owner import device_limits, device_state, digest, make_inference_runner, check_gripper_policy, episode_duration, execution_steps
from .inference_transport import MODEL_ID, RemoteBaseClient, ModelSelectionChanged
from .inference_history import InferenceHistory
from .sample_clock import clock_spec, sample_stamp, POLICY_DURATION_TOLERANCE_S
from .wam import HORIZON as WAM_HORIZON

CAMERAS = {'head':'base_0', 'left':'left_wrist_0', 'right':'right_wrist_0'}
METRIC_EVENTS=frozenset(('prediction_received','prediction_discarded','prediction_requested',
    'bounded_prediction_result','finite_chunk_prepared','bounded_prediction_prepared',
    'finite_chunk_started','normal_model_segment_started','episode_started','episode_completed','episode_stopped'))


class _Run(dict):
    """Weak-referenceable run state; callbacks must not retain retired runs."""


def calibrated_adapter(profile, sdk_limits, *, model_id=MODEL_ID, disabled_grippers=()):
    if model_family(model_id)=='wam':
        from .wam import Adapter,validate_native_profile
        from arx_env.joint_head_jaw_mapping import mapping_identity
        native=validate_native_profile(profile)['identity']
        if disabled_grippers or mapping_identity().get('native_calibration_id')!=native:
            raise ValueError('WAM requires both measured jaws and the active native SDK calibration')
        maximum=profile['gripper']['max_opening_mm']
        return Adapter(device_limits(sdk_limits,native_coordinates=True),maximum),dict(joint_offsets_rad=[0.]*12,
            native_calibration_id=native,model_jaw_unit='recorder_endpoint_m',device_jaw_unit='recorder_endpoint_mm',
            model_jaw_formula='device_mm / 1000',profile_max_opening_mm=maximum,
            arms='both',history_steps=0,input_jaw_clipping=False,position_projection=True,extrinsics_consumed=False)
    if model_family(model_id)=='mixed366':
        from .mixed366_control import adapter
        return adapter(profile,sdk_limits,disabled_grippers)
    from joint_space import offsets, gripper_offset_mm
    zero = np.asarray(offsets(profile), dtype=np.float64)
    gap_offset_mm = gripper_offset_mm(profile)
    joint_slots = [*range(6), *range(7,13)]
    def to_model(value):
        result = vector14(value).copy()
        result[joint_slots] += zero
        result[[6,13]] = (result[[6,13]]-gap_offset_mm)/1000.
        return result
    def to_device(value):
        result = vector14(value).copy()
        result[joint_slots] -= zero
        result[[6,13]] = result[[6,13]]*1000.+gap_offset_mm
        return result
    limits = device_limits(sdk_limits)
    if tuple(disabled_grippers) not in ((),('right',)):raise ValueError('only the right gripper may be disabled for inference')
    assumed=(profile['gripper']['max_opening_mm']-gap_offset_mm)/1000. if disabled_grippers else None
    if disabled_grippers:limits[13]=[0.,profile['gripper']['max_opening_mm']]
    opened = min(83., profile['gripper']['max_opening_mm'], limits[6][1])
    if model_family(model_id)=='base566':
        adapter=ContinuousBaseAdapter(model_to_device=to_device,device_to_model=to_model,position_limits=limits,
            project_position_limits=True,disabled_right_gripper_model_m=assumed)
    else:
        adapter = LegacyBaseAdapter(model_to_device=to_device, device_to_model=to_model,
            position_limits=limits, closed_jaw_mm=65., open_jaw_mm=opened, project_position_limits=True)
    identity = dict(joint_model_equals_device_plus_zero_rad=zero.tolist(), model_gap_offset_mm=gap_offset_mm,
        model_jaw_unit='m', device_jaw_unit='mm', model_jaw_formula='(device_mm - offset_mm) / 1000',
        profile_max_opening_mm=profile['gripper']['max_opening_mm'],disabled_grippers=list(disabled_grippers),
        right_arm_policy='hold_observed_at_episode_start', position_projection=True)
    if model_family(model_id)=='base566':identity['jaw_encoding']='continuous_sum_two_finger_travel_m'
    else:identity.update(closed_jaw_mm=65.,open_jaw_mm=opened,legacy_binary_threshold_m=.044)
    if disabled_grippers:
        identity.update(disabled_right_gripper_model_m=assumed,disabled_right_gripper_device_mm=profile['gripper']['max_opening_mm'],
            disabled_right_gripper_assumption='profile_max_opening_not_measured',disabled_right_sdk_output=False)
    return adapter, identity


def physical14(values):
    values = vector14(values).tolist()
    return dict(schema_version='physical14.v1', encoding='absolute', units=dict(joints='rad', grippers='mm'),
        left_arm_joints=values[:6], left_gripper=values[6], right_arm_joints=values[7:13], right_gripper=values[13])


class InferenceWorkflow:
    def __init__(self, device, output, *, camera_service, model_endpoint, model_token_file,
                 profile_provider, support_provider=None, event=None, model_id=MODEL_ID, model_bundle=None,
                 model_catalog_provider=None, catalog_root=None, disabled_grippers=(), recording_enabled=True,
                 model_prepare_operation=None, reference_trajectory=None,prepared_reference_execution=False,reference_integral_feedback=False,
                 control_mapping_pending=None, task=None, rgb_source=None, action_chunks=False, normal_modes=None,
                 normal_initial_mode='MODEL',normal_finish_kind=None,normal_finish_policy=None,reference_planning=None,
                 finite_chunk_count=None,recording_backend=None,ros_plane=None,wam_stall_recovery_available=True):
        if type(wam_stall_recovery_available)is not bool:raise ValueError('explicit WAM trick availability required')
        self.wam_stall_recovery_available=wam_stall_recovery_available
        if type(recording_enabled) is not bool:raise ValueError('recording_enabled must be boolean')
        if recording_backend not in (None,'capture_session_v1'):raise ValueError('known recording backend required')
        self.recording_backend=recording_backend
        from .execution_settings import control_parameters,defaults,policy_hash
        input_defaults=control_parameters(defaults('inference'),business='inference')
        self._input_default_policy=dict(revision=None,policy_hash=policy_hash(input_defaults),parameters=input_defaults)
        if type(action_chunks)is not bool:raise ValueError('action_chunks must be boolean')
        if normal_modes is not None and (not isinstance(normal_modes,dict) or not action_chunks
                or recording_enabled or normal_modes.get('recording_required')is not False):
            raise ValueError('workbench normal modes require action chunks and recording disabled; Raw gate belongs to the collector')
        if (action_chunks or normal_modes is not None) and reference_trajectory is not None:
            raise ValueError('live action chunks cannot replace a frozen research reference')
        self.action_chunks,self.normal_modes=action_chunks,deepcopy(normal_modes)
        from .ros_data import plane_config
        self.ros_plane=plane_config(ros_plane) if ros_plane is not None else None
        self.ros_node=self.ros_inputs=self.ros_worker=None
        self.reference_planning=None
        if reference_planning is not None:
            if not action_chunks or model_family(model_id)not in('mixed366','wam') or reference_trajectory is not None:
                raise ValueError('reference planning requires recorded-native live action chunks')
            from .motion_reference import infer_reference_policy
            self.reference_planning=infer_reference_policy(reference_planning)
        if finite_chunk_count is not None and (type(finite_chunk_count)is not int or finite_chunk_count not in (3,4)
                or self.reference_planning is None or recording_enabled or normal_modes is not None):
            raise ValueError('finite workbench acceptance requires three or four complete planned chunks and recording disabled')
        self.finite_chunk_count=finite_chunk_count
        if normal_initial_mode not in ('MODEL','HOLD')or normal_initial_mode=='HOLD'and normal_modes is None:
            raise ValueError('initial HOLD requires explicitly configured normal modes')
        if (normal_finish_kind is not None or normal_finish_policy is not None)and(
                normal_modes is None or normal_finish_kind!='maintenance_home_v1' or not isinstance(normal_finish_policy,dict)):
            raise ValueError('workbench finish requires explicit maintenance_home_v1 policy and normal modes')
        self.normal_initial_mode=normal_initial_mode
        self.normal_finish_kind,self.normal_finish_policy=normal_finish_kind,deepcopy(normal_finish_policy)
        self.recording_enabled=recording_enabled
        self.control_mapping_pending=control_mapping_pending
        if task is not None and task not in ('coffee','lid_handoff'):raise ValueError('registered training task required')
        self.task,self.rgb_source=task,rgb_source
        from .inference_reference import load_reference
        self.reference_pin=deepcopy(reference_trajectory)
        self.reference=load_reference(reference_trajectory) if reference_trajectory is not None else None
        if type(prepared_reference_execution)is not bool or prepared_reference_execution and self.reference is None:
            raise ValueError('prepared execution requires an explicit frozen research reference')
        self.prepared_reference_execution=prepared_reference_execution
        if type(reference_integral_feedback)is not bool or reference_integral_feedback and not prepared_reference_execution:
            raise ValueError('reference integral requires a prepared research reference')
        self.reference_integral_feedback=reference_integral_feedback
        self.device, self.output = device, Path(output)
        self.cameras, self.profile_provider = camera_service, profile_provider
        self.model_endpoint, self.model_token_file = model_endpoint, model_token_file
        self.support_provider, self.event = support_provider, event
        self.selected_model_id,self.model_bundle=model_id,deepcopy(model_bundle)
        self.selected_spec=model_spec(model_id,model_bundle)
        self.selected_bundle_digest=model_bundle_digest(model_id,model_bundle)
        self.model_catalog_provider,self.catalog_root=model_catalog_provider,catalog_root
        self.models=[dict(id=model_id,label=self.selected_spec['label'],epoch=self.selected_spec['epoch'],
                         step=self.selected_spec['step'],selectable=True,status='configured',reason=None,
                         backend=self.selected_spec.get('backend','torch'),bundle_digest=self.selected_bundle_digest)]
        self.model_descriptors={self.selected_bundle_digest:deepcopy(model_bundle)}
        self.catalog_error=None
        if tuple(disabled_grippers) not in ((),('right',)):raise ValueError('only right gripper may be disabled')
        self.disabled_grippers=tuple(disabled_grippers)
        self.lock = threading.RLock()
        self.observation_lock=threading.Lock()
        from .observation_pairing import TimestampPairer
        from .inference_evidence import InferenceEvidence
        self.observation_pairer=TimestampPairer()
        self.input_evidence=InferenceEvidence(self.output/'native-inputs')if self.ros_plane is None else None
        self.generation = 0
        self.instance_id=secrets.token_hex(16)
        self.seed = 0
        self.duration_s = .5 if finite_chunk_count is not None else 120.
        self.status, self.stage, self.message = 'idle', 'inference.idle', '选择模型后加载与预热；准备现场后再运行。'
        self.busy = False
        self.client = self.adapter = self.prepared = self.current = None
        self.pending_captures={}
        self.model_release_unconfirmed=False
        self.model_client=None
        self.model_identity = None
        self.model_prepare_operation=model_prepare_operation
        self.model_operation=None
        self.model_status,self.model_error,self.model_observed_wall_ns='not_loaded',None,None
        self.model_service_fact=dict(status='unknown',loaded=None)
        self.model_fact_received=None
        self.model_binding_identity={}
        self.model_probe_stop=threading.Event()
        self.cache = self.support_cache = None
        self.cache_error = '尚未取得三相机与反馈的配对观测'
        self.cache_stop = threading.Event()
        self.cache_thread = None
        self.cache_paused = False
        self.output.mkdir(parents=True, exist_ok=True)
        settings_path=self.output/'execution-settings.json'
        settings=json.loads(settings_path.read_text()) if settings_path.exists() else {}
        self.execute_steps=execution_steps(settings.get('execute_steps',15))
        if finite_chunk_count is not None:self.execute_steps=15
        if self.reference_planning is not None:self.reference_planning['max_execution_duration_s']=self.execute_steps/30.*4
        self.execution_revision=settings.get('revision',0)
        if type(self.execution_revision) is not int or self.execution_revision<0:raise ValueError('invalid inference settings revision')
        self.history = InferenceHistory(self.output)
        self.capture_results=None
        self.debug_request=None;self.capture_requests={};self.artifacts=None
        if self.recording_backend:
            from .inference_trace import RunArtifacts,CaptureRun
            self.capture_factory=CaptureRun
            if self.ros_plane is None:self.artifacts=RunArtifacts(self.output,self.input_evidence,self._debug_finished)
            from .collection_history import CaptureResults
            self.capture_results=CaptureResults(self.output,environment=self.device.environment,device_id=self.device.id,inference_history=self.history)
        self.model_probe_thread=threading.Thread(target=self._model_fact_loop,daemon=True,name='model-facts')
        self.model_probe_thread.start()
        self.model_catalog_thread=threading.Thread(target=self._model_catalog_loop,daemon=True,name='model-catalog')
        self.model_catalog_thread.start()

    def begin_debug_request(self,job,body):
        if self.artifacts is None:return None
        try:handle=self.artifacts.begin(job,body)
        except Exception as error:
            handle=dict(job_id=job['id'],request={},started_wall_ns=job['started_wall_ns'],execution_instance_id=None,
                        model_requests=[],status='running',artifact_error=str(error))
        self.debug_request=handle
        return handle

    def finish_debug_request(self,handle,job,runtime):
        capture=self.capture_requests.get(handle['job_id'])
        if capture is None:self.artifacts.finish(handle,job,runtime)
        else:
            def finish_request():
                capture.done.wait(100)
                path=capture.directory/'debug-receipt.json'
                if path.is_file():
                    value=json.loads(path.read_text());value['application_status']=job['status']
                    self.artifacts._request_receipt(value)
                self.capture_requests.pop(handle['job_id'],None)
            self.artifacts._submit(finish_request)
        if self.debug_request is handle:self.debug_request=None

    def _refresh_model_fact(self):
        with self.lock:client,generation=self.model_client,self.generation
        if client is None:return
        try:
            value=client.status()
            if not isinstance(value,dict) or type(value.get('loaded')) is not bool:
                raise ValueError('invalid selected model status')
            value={key:deepcopy(value.get(key)) for key in ('status','loaded','model_id','warmup',
                'service_pid','worker_pid','worker_exit_code','fact','error')}
        except ModelSelectionChanged as error:value=dict(status='selection_changed',loaded=False,error=str(error))
        except Exception as error:value=dict(status='unknown',loaded=None,error=str(error))
        with self.lock:
            if self.generation==generation and self.model_client is client:
                self.model_service_fact=value;self.model_fact_received=time.monotonic()

    def _model_fact_loop(self):
        while not self.model_probe_stop.wait(1.):self._refresh_model_fact()

    def _model_catalog_loop(self):
        # Catalog IO has its own wait so a slow scan cannot stale model health.
        while not self.model_probe_stop.wait(5.):
            try:
                self.refresh_models()
            except Exception:
                pass  # refresh_models retains the last catalog and exposes catalog_error.

    def _model_gate(self):
        from .state_contract import model_readiness
        if self.model_identity is None:
            return dict(ready=False,process_ready=None,workspace_bound=False,reason='请先选择并准备模型')
        service=self.model_service_fact
        if self.model_fact_received is None or time.monotonic()-self.model_fact_received>3.:
            service=dict(status='unknown',loaded=None,error='所选模型状态已过期，等待模型服务核验')
        return model_readiness(dict(selected_model_id=self.selected_model_id,
            model_ready=self.model_identity is not None and self.model_status=='ready',
            **self.model_binding_identity),service)

    def _control_ready(self):
        runtime=self.device.runtime
        if (self.prepared is None or self.cache is None or self.cache_error or runtime is None
                or runtime.holder!='inference' or runtime.owner.feedback_healthy is not True
                or not self._binding_current(self.prepared)):return False
        try:
            return self._observe(runtime.owner.control_time(),runtime.owner.cache.get('calibration_feedback')or{},
                self.prepared['binding_digest'])is not None
        except (KeyError,TypeError,ValueError):return False

    def _binding_current(self,prepared):
        healthy=getattr(self.client,'healthy',None)
        if callable(healthy) and not healthy():return False
        runtime=self.device.runtime
        if runtime is None:return False
        expected=prepared.get('dependencies')
        return expected is None or expected==self._device_dependencies()

    def _device_dependencies(self):
        runtime=self.device.runtime
        return dict(runtime_nonce=runtime.owner.runtime_nonce if runtime else None,
            calibration_revision=getattr(runtime,'calibration_revision',None),
            calibration_snapshot_id=getattr(runtime,'calibration_snapshot_id',None),
            camera_service_instance=(getattr(self.cameras,'descriptor',{})or{}).get('instance_id'))

    def execution_settings(self):
        with self.lock:
            self._sync_control_policy()
            busy=self.snapshot()['busy']
            return dict(execute_steps=self.execute_steps,revision=self.execution_revision,
                action_horizon=WAM_HORIZON if str(getattr(self,'selected_model_id','')).startswith('wam-') else 60,
                control_hz=30,allowed=not busy and self.finite_chunk_count is None,
                reason='当前验证固定每轮完整15步'if self.finite_chunk_count is not None else'请等待本轮推理或处理结束'if busy else None)

    def _policy_owner(self):
        runtime=getattr(getattr(self,'device',None),'runtime',None)
        if runtime is None or not getattr(runtime,'supports_control_policy',False):return None
        if getattr(runtime,'remote_control_owner',False) and not runtime.state.get('runtime_nonce'):return None
        return runtime

    def _sync_control_policy(self):
        runtime=self._policy_owner()
        if runtime is None:return
        state=getattr(runtime,'state',{})
        revision=getattr(runtime,'execution_settings_revision',None)
        if revision is None:revision=(state.get('execution_settings')or{}).get('revision')
        key=(id(runtime),getattr(runtime,'nonce',state.get('runtime_nonce')),revision,
             getattr(runtime,'inference_revision',state.get('inference_configuration_revision')),getattr(self,'generation',0))
        if getattr(self,'_policy_cache_key',None)==key:return
        policy=runtime.control_policy_snapshot('inference')
        self._input_policy_snapshot=policy
        from .execution_settings import Guard,TRAINING_RGB_PAIR_ERROR_S,TRAINING_STATE_PAIR_ERROR_S
        if getattr(self,'observation_pairer',None) is not None:
            self.observation_pairer.guard=Guard.bind(policy,dict(training_rgb_pair_error_s=TRAINING_RGB_PAIR_ERROR_S,
                                                               training_state_pair_error_s=TRAINING_STATE_PAIR_ERROR_S))
        self._policy_cache_key=key
        self.execution_revision=policy['revision']
        parameters=policy['parameters']
        if policy['origins'].get('execution.execute_steps')!='product_default':
            self.execute_steps=parameters['execution']['execute_steps']
        if self.reference_planning is not None:
            self.reference_planning.update(parameters.get('planning',{}),max_execution_duration_s=self.execute_steps/30.*4)
            if 'planning'not in parameters and self.device.environment=='physical'and parameters['measured']['enabled']:
                self.reference_planning['velocity_limit_rad_s']=min(
                    self.reference_planning['velocity_limit_rad_s'],parameters['measured']['limit_rad_s'])
            if getattr(self,'normal_modes',None) is not None:
                self.normal_modes.update(velocity_limit=self.reference_planning['velocity_limit_rad_s'],
                    acceleration_limit=self.reference_planning['acceleration_limit_rad_s2'])
        self._control_policy_parameters=deepcopy(parameters)

    def set_execution_steps(self, value, expected_revision):
        value=execution_steps(value,horizon=WAM_HORIZON if str(getattr(self,'selected_model_id','')).startswith('wam-') else 60)
        if self.finite_chunk_count is not None and value!=15:raise ValueError('当前验证固定每轮完整15步')
        with self.lock:
            if self.snapshot()['busy']:raise ValueError('请等待本轮推理或处理结束')
            runtime=self._policy_owner()
            if runtime is not None:
                self._sync_control_policy()
                if type(expected_revision) is not int or expected_revision!=self.execution_revision:
                    raise ValueError('推理执行设置已变化，请重新读取后应用')
                policy=runtime.update_control_policy(self.device.credentials['inference'],
                    dict(expected_revision=expected_revision,changes={'execution.execute_steps':value}))
                self._policy_cache_key=None
                self.execute_steps=value;self.execution_revision=policy['revision']
                self.invalidate()
                return self.execution_settings()
            if type(expected_revision) is not int or expected_revision!=self.execution_revision:
                raise ValueError('推理执行设置已变化，请重新读取后应用')
            self._write_json(self.output/'execution-settings.json',dict(execute_steps=value,revision=self.execution_revision+1))
            self.execute_steps=value;self.execution_revision+=1
            if self.reference_planning is not None:self.reference_planning['max_execution_duration_s']=value/30.*4
            self.invalidate()
            return self.execution_settings()

    def _write_json(self, path, value):
        temporary = path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)+'\n')
        os.replace(temporary, path)

    def _new_client(self, adapter=None, conversion=None):
        if model_family(self.selected_model_id)=='wam' and adapter is not None:
            adapter.bind_normalization(self.model_bundle['normalization_json'].encode(),
                expected_sha256=self.model_bundle['artifacts']['stats']['sha256'])
        return RemoteBaseClient(self.model_endpoint,token_file=self.model_token_file,adapter=adapter,
            conversion_identity=conversion or dict(scope='model_preparation_only',device_id=self.device.id,
                environment=self.device.environment,disabled_grippers=list(self.disabled_grippers)),
            model_id=self.selected_model_id,model_bundle=self.model_bundle,
            **(dict(task=self.task)if model_family(self.selected_model_id)=='mixed366'else{}))

    def refresh_models(self):
        """List remote metadata only; never scan/download checkpoints on the device."""
        try:
            client=None
            if self.model_catalog_provider is not None:value=self.model_catalog_provider()
            else:
                client=self._new_client();value=client.list_models()
            rows=value.get('models',[]) if isinstance(value,dict) else value
            if not isinstance(rows,list):raise ValueError('模型目录列表格式无效')
            public=[];descriptors={};seen=set()
            for index,row in enumerate(rows):
                if not isinstance(row,dict):raise ValueError('模型目录条目格式无效')
                identity=row.get('model_id',row.get('id'))
                selectable=row.get('selectable') is True
                descriptor=row.get('model_descriptor')
                backend=(descriptor or {}).get('backend',row.get('backend','torch'))
                from .wam import is_model_id
                allowed_backends=('torch','anyinfer')if is_model_id(identity) else('torch','triton')
                if selectable and (backend not in allowed_backends or row.get('backend',backend)!=backend):
                    raise ValueError('模型目录模式与描述不一致')
                if not isinstance(identity,str) or not identity:
                    if selectable:raise ValueError('可选模型目录身份缺失或重复')
                    identity='unavailable-'+digest(dict(index=index,label=row.get('label'),epoch=row.get('epoch'),step=row.get('step')))[:16]
                if selectable:
                    model_spec(identity,descriptor)
                    bundle_digest=model_bundle_digest(identity,descriptor)
                    if row.get('bundle_digest',bundle_digest)!=bundle_digest:raise ValueError('模型目录摘要与描述不一致')
                else:bundle_digest=row.get('bundle_digest')
                selection_key=(identity,backend,bundle_digest)
                if selectable and selection_key in seen:raise ValueError('可选模型目录身份重复')
                seen.add(selection_key)
                if selectable:descriptors[bundle_digest]=deepcopy(descriptor)
                public.append(dict(id=identity,backend=backend,selectable=selectable,bundle_digest=bundle_digest,
                    runtime_schema=(descriptor or {}).get('schema'),
                    **{key:deepcopy(descriptor[key])for key in ('inference_options','inference_capabilities')if key in(descriptor or{})},
                    **{key:deepcopy(row.get(key)) for key in ('label','epoch','step','status','reason')}))
            with self.lock:
                self.models,self.model_descriptors,self.catalog_error=public,descriptors,None
                if isinstance(value,dict) and value.get('catalog_root') is not None:self.catalog_root=value['catalog_root']
            return dict(models=deepcopy(public),selected_model_id=self.selected_model_id,catalog_root=self.catalog_root)
        except Exception as error:
            with self.lock:self.catalog_error=str(error)
            raise
        finally:
            if 'client' in locals() and client is not None:client.close()

    def _revoke_device_review(self):
        runtime=self.device.runtime
        if runtime is None:return
        runtime.revoke_inference()

    def _model_selection(self, model_id, expected_bundle_digest=None):
        with self.lock:
            if self.snapshot()['busy']:raise ValueError('推理或数据处理未结束，不能切换模型')
        self.refresh_models()
        with self.lock:
            candidates=[row for row in self.models if row['id']==model_id
                        and (expected_bundle_digest is None or row['bundle_digest']==expected_bundle_digest)]
            if expected_bundle_digest is not None and not candidates and any(row['id']==model_id for row in self.models):
                raise ValueError('模型目录身份已变化，请重新选择')
            if len(candidates)>1:raise ValueError('同一checkpoint存在多个推理模式，请明确选择模式与目录摘要')
            row=candidates[0]if candidates else None
            if row is None or row.get('selectable') is not True:
                raise ValueError((row or {}).get('reason') or '模型尚不可选或已不在目录中')
            if expected_bundle_digest is not None and row.get('bundle_digest')!=expected_bundle_digest:
                raise ValueError('模型目录身份已变化，请重新选择')
            descriptor=deepcopy(self.model_descriptors[row['bundle_digest']])
        return model_id,descriptor,row['bundle_digest']

    def _request_model_prepare(self, parameters):
        callback=self.model_prepare_operation
        if callback is None:raise ValueError('模型准备需要运维 Operation；当前部署未接入统一运维入口')
        result=callback(parameters)
        with self.lock:self.model_operation=deepcopy(result)
        return result

    def select_model(self, model_id, expected_bundle_digest=None):
        model_id,_,bundle_digest=self._model_selection(model_id,expected_bundle_digest)
        return self._request_model_prepare(dict(model_id=model_id,expected_bundle_digest=bundle_digest))

    def prepare_model(self, *, seed=None, force=False, expected_model_id=None):
        """Compatibility entry: request the same visible management operation."""
        if seed is not None:
            if type(seed) is not int or not 0<=seed<2**32:raise ValueError('本轮需要有效 uint32 随机种子')
            with self.lock:self.seed=seed
        if expected_model_id is not None and expected_model_id!=self.selected_model_id:
            raise ValueError('模型选择已变化，请核验原模型并显式选择')
        return self._request_model_prepare(dict(model_id=self.selected_model_id,expected_bundle_digest=self.selected_bundle_digest))

    def environment_status(self):
        """Memory-only capability facts; never parse an episode/business snapshot."""
        with self.lock:
            from .state_contract import fact
            gate=self._model_gate();ready=gate['ready'] is True
            runner=(self.current or {}).get('runner')
            capture=(self.current or{}).get('recorder')
            active=getattr(runner,'status',None) in ('ARMED','PREPARING','VERIFYING','RUNNING','SETTLING')
            operator=getattr(self,'operator_task',{})
            active=active or operator.get('status')=='running'
            return dict(selected_model_id=self.selected_model_id,bundle_digest=self.selected_bundle_digest,
                backend=self.selected_spec.get('backend','torch'),
                busy=bool(self.busy or active or getattr(runner,'recording_pending',False)
                    and not getattr(capture,'ingress_closed',False)),stage_id=self.stage,
                loaded=ready,ready=ready,model_ready=ready,generation=self.generation,
                workspace_bound=self.model_identity is not None,readiness=gate,
                control_ready=self._control_ready(),
                binding_fact=fact('inference_binding','InferenceWorkflow',self.instance_id,self.generation,
                    self.model_observed_wall_ns,dependencies=dict(model=self.model_binding_identity,
                        device=(self.prepared or {}).get('dependencies')),valid=ready,reason=gate['reason']),
                **self.model_binding_identity,
                status=self.model_status,error=self.model_error,observed_wall_ns=self.model_observed_wall_ns,
                operation=deepcopy(self.model_operation))

    def operations_prepare_model(self, model_id=None, expected_bundle_digest=None, expected_model_id=None, *, still_current):
        """Only a live, scoped maintenance operation may load or reuse a model."""
        def current():return bool(getattr(self.device,'maintenance',False) and still_current())
        if not callable(still_current) or not current():raise ValueError('有效运维维护窗口才能准备模型')
        selection=None
        if model_id is not None:
            identity,descriptor,_=self._model_selection(model_id,expected_bundle_digest)
            selection=(identity,descriptor)
        elif expected_bundle_digest is not None and expected_bundle_digest!=self.selected_bundle_digest:
            raise ValueError('模型目录身份已变化，请重新选择')
        return self._prepare_model(_selection=selection,expected_model_id=expected_model_id,still_current=current)

    def operations_invalidate_model(self,*,still_current):
        """Revoke local bindings only; the manager owns actual worker retirement."""
        def current():return bool(getattr(self.device,'maintenance',False) and still_current())
        if not callable(still_current) or not current():raise ValueError('有效运维维护窗口才能撤销模型绑定')
        with self.lock:
            if self.snapshot()['busy']:raise ValueError('推理或录制尚未结束，不能撤销模型绑定')
        self._revoke_device_review()
        with self.lock:
            if not current():raise ValueError('模型绑定撤销的运维操作已失效')
            clients=[self.client,self.model_client]
            self.generation+=1
            self.client=self.model_client=self.adapter=self.prepared=self.current=None
            self.cache=self.support_cache=self.model_identity=None
            self.cache_error='模型绑定已由运维撤销；选择并准备模型后重新核对观测'
            self.model_status,self.model_error,self.model_observed_wall_ns='not_loaded',None,time.time_ns()
            self.status,self.stage,self.message='idle','inference.idle','模型绑定已撤销，未连接设备或恢复旧执行。'
            generation=self.generation
        from .model_session import PinnedModelClient
        pinned=any(isinstance(client,PinnedModelClient)and client.token is not None for client in clients)
        try:
            for index,client in enumerate(clients):
                if client is not None and not any(client is old for old in clients[:index]):client.close()
        except Exception:
            self.model_release_unconfirmed=True
            raise
        return dict(invalidated=True,generation=generation,model_ready=False,worker_lifecycle_changed=False,
            pin_release=dict(status='unconfirmed'if self.model_release_unconfirmed else 'released'if pinned else 'not_acquired'))

    def _prepare_model(self, *, _selection=None, expected_model_id=None, still_current):
        """Synchronous model-only load/warmup; never connects or commands a device."""
        with self.lock:
            if self.snapshot()['busy']:raise ValueError('推理任务仍在处理')
            if expected_model_id is not None and (expected_model_id!=self.selected_model_id or _selection is not None):
                raise ValueError('重载后模型选择已变化，请核验原模型并显式选择；未自动加载或切换模型')
            self.busy=True
        candidate=None
        try:
            if not still_current():raise ValueError('模型准备运维操作已撤销')
            self._revoke_device_review()
            with self.lock:
                if _selection is not None:
                    self.selected_model_id,self.model_bundle=_selection
                    self.selected_spec=model_spec(*_selection)
                    self.selected_bundle_digest=model_bundle_digest(*_selection)
                self.generation+=1;generation=self.generation
                prior=[self.client,self.model_client]
                self.client=self.model_client=self.adapter=self.prepared=self.current=None
                self.cache=self.support_cache=None;self.model_identity=None
                self.cache_error='模型准备不连接设备；运行前需要新鲜反馈与三相机'
                self.status='loading';self.stage='inference.prepare';self.message='正在加载与预热 '+self.selected_spec['label']+'；不执行动作。'
                self.model_status,self.model_error='loading',None
            for client in prior:
                if client is not None:client.close()
            if model_family(self.selected_model_id)=='wam':
                from .wam import _torch
                _torch()  # Verify the business-side CPU preprocessor before warmup.
            candidate=self._new_client()
            if not still_current():raise ValueError('模型准备运维操作已撤销')
            try:identity=candidate.prepare(ready_only=True);reused=True
            except Exception:
                if expected_model_id is not None:raise
                if not still_current():raise ValueError('模型准备运维操作已撤销')
                identity=candidate.prepare();reused=False
            with self.lock:
                if self.generation!=generation or not still_current():raise ValueError('模型准备已撤销，未绑定到设备')
                self.model_client,self.model_identity=candidate,identity
                self.model_binding_identity={}
                self.model_status,self.model_error,self.model_observed_wall_ns='ready',None,time.time_ns()
                self.status='model_ready';self.stage='inference.idle';self.message='所选模型已预热；设备执行尚未准备。'
            self._refresh_model_fact()
            with self.lock:
                if self._model_gate()['ready'] is not True:raise ValueError(self._model_gate()['reason'])
                self.model_binding_identity=dict(bound_service_pid=self.model_service_fact.get('service_pid'),
                    bound_worker_pid=self.model_service_fact.get('worker_pid'),
                    bound_model_instance=(self.model_service_fact.get('fact')or{}).get('instance_id'))
            return dict(message=self.message,model_ready=True,control_ready=False,
                        model_id=self.selected_model_id,bundle_digest=self.selected_bundle_digest,
                        backend=self.selected_spec.get('backend','torch'),generation=generation,reused=reused)
        except Exception as error:
            if candidate is not None:candidate.close()
            with self.lock:
                self.status='failed';self.message=str(error)
                self.model_status,self.model_error,self.model_observed_wall_ns='error',str(error),time.time_ns()
            raise
        finally:
            with self.lock:self.busy=False

    def bind_device(self):
        """Bind a warm model to already-connected fresh device feedback; no acquisition or motion."""
        with self.lock:
            if self.snapshot()['busy']:raise ValueError('推理任务仍在处理')
            if self._model_gate()['ready'] is not True:raise ValueError(self._model_gate()['reason'])
            self.generation+=1;generation=self.generation;self.busy=True
        client=None
        try:
            if self.device.runtime is None or self.device.runtime.holder!='inference':
                raise ValueError('请先连接设备并交接给推理业务')
            profile,binding=self._binding()
            if self.ros_plane is not None:self._bind_ros_sources(binding)
            _,_,_,limits=self._feedback()
            adapter,mapping=calibrated_adapter(profile,limits,model_id=self.selected_model_id,disabled_grippers=self.disabled_grippers)
            conversion=dict(device_id=self.device.id,environment=self.device.environment,binding=binding,mapping=mapping,
                disabled_grippers=list(self.disabled_grippers))
            client=self._new_client(adapter,conversion);identity=client.prepare(ready_only=True)
            if callable(getattr(client,'pin',None)):
                client=client.pin('workbench-'+self.instance_id+'-'+str(generation))
            chunk_binding=None
            if self.action_chunks:
                from .inference_model import action_contract
                model_binding=deepcopy(getattr(client,'model_binding',None))
                contract=deepcopy(getattr(client,'action_contract',None))
                if (not callable(getattr(client,'infer_chunk',None)) or not isinstance(model_binding,dict)
                        or model_binding.get('model_id')!=self.selected_model_id
                        or model_binding.get('bundle_digest')!=self.selected_bundle_digest
                        or contract!=action_contract(self.selected_model_id,self.model_bundle)
                        or not isinstance(getattr(client,'session_id',None),str) or not client.session_id):
                    raise ValueError('prepared model lacks the trusted complete action chunk binding')
                chunk_binding=dict(session_id=client.session_id,model_binding=model_binding,
                    conversion_id=client.conversion_id,action_contract=contract,
                    task=self.task if model_family(self.selected_model_id)=='mixed366' else self.selected_spec['prompt'])
            if self._binding()!=(profile,binding):raise ValueError('模型准备期间标定或相机绑定已变化，请重新准备')
            _,_,_,current_limits=self._feedback()
            if current_limits!=limits:raise ValueError('设备限位已变化，请重新准备模型接入')
            with self.lock:
                if self.generation!=generation:raise ValueError('模型准备已撤销，未绑定到设备')
                old=self.client
                self.client,self.adapter,self.model_identity=client,adapter,identity
                self.prepared=dict(profile=profile,binding=binding,binding_digest=digest(binding),conversion=conversion,
                    conversion_id=client.conversion_id,limits=limits,disabled_grippers=list(self.disabled_grippers),
                    model_id=self.selected_model_id,dependencies=self._device_dependencies(),capture_instances=None)
                if chunk_binding is not None:self.prepared['action_chunk_binding']=chunk_binding
                self.cache=None;self.support_cache=None;self.current=None
                self.status='ready';self.stage='inference.idle';self.message='模型已绑定设备，等待新鲜三相机后准备执行审阅。'
            if old is not None:old.close()
            if self.ros_plane is not None:self._prepare_ros_model()
            self._start_cache()
            return dict(message=self.message,model_ready=True,device_bound=True,generation=generation)
        except Exception as error:
            if client is not None:client.close()
            with self.lock:self.prepared=None;self.cache=None;self.status='failed';self.message=str(error)
            raise
        finally:
            with self.lock:self.busy=False

    def preview_episode(self, seed=0, duration_s=120,reference_feedforward_sha256=None,*,wam_stall_recovery=False):
        if type(seed) is not int or not 0<=seed<2**32:raise ValueError('本轮需要有效 uint32 随机种子')
        duration_s=episode_duration(duration_s)
        if self.finite_chunk_count is not None and duration_s!=.5:raise ValueError('每轮模型源轨迹固定0.5秒15步')
        self._wait_observation()
        return self._preview(seed,duration_s,reference_feedforward_sha256,wam_stall_recovery=wam_stall_recovery)

    def execute_review(self, token, physical_clearance=True):
        with self.lock:
            if self.status!='awaiting_review':raise ValueError('本轮推理审阅已撤销，请重新准备')
            return self.device.submit('inference',dict(action='confirm',token=token,
                confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=physical_clearance is True))

    def _binding(self):
        from integration import _digest, validate_profile
        profile, binding = self.profile_provider()
        profile, binding = deepcopy(profile), deepcopy(binding)
        if model_family(self.selected_model_id)=='wam':
            from .wam import validate_native_profile,NATIVE_SCHEMA
            validate_native_profile(profile)
            expected=dict(schema=NATIVE_SCHEMA,robot_id=self.device.id,camera_serials=profile['camera_serials'],
                calibration_profile_digest=digest(profile),coordinate_qualification='recorded_native_state14',extrinsics_consumed=False)
            if profile['robot_id']!=self.device.id or binding!=expected:
                raise ValueError('WAM native device/camera binding differs')
            return profile,binding
        if model_family(self.selected_model_id)=='mixed366':
            from .mixed366_control import validate_profile,SCHEMA
            validate_profile(profile)
            if (self.task not in ('coffee','lid_handoff') or binding.get('schema')!=SCHEMA
                    or binding.get('robot_id')!=self.device.id or self.device.environment!='physical'
                    or binding.get('calibration_profile_digest')!=digest(profile)
                    or binding.get('camera_serials')!=profile['camera_serials']):
                raise ValueError('mixed366 native device/task binding differs')
            return profile,binding
        validate_profile(profile)
        serials = binding.get('camera_serials') or {}
        if (binding.get('calibration_accepted') is not True or binding.get('robot_id') != self.device.id
                or set(serials) != set(CAMERAS) or len(set(serials.values())) != 3
                or binding.get('calibration_profile_digest') != _digest(profile)
                or bool(profile['source'].get('simulation_only')) != (self.device.environment == 'fake')):
            raise ValueError('当前设备、三相机与已安装标定绑定不完整或不一致')
        return profile, binding

    def _input_guard(self,rule,value,source_time,checked_time,identity,*,realtime=False,expected_identity=None):
        from .execution_settings import Guard,control_parameters,defaults,policy_hash
        if not realtime:self._sync_control_policy()
        current=getattr(self,'current',None)
        if current is not None and current.get('ended_wall')is not None:current=None
        policy=(current or{}).get('control_policy') or getattr(self,'_input_policy_snapshot',None)
        if policy is None:
            policy=getattr(self,'_input_default_policy',None)
            if policy is None:
                parameters=control_parameters(defaults('inference'),business='inference')
                policy=dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters)
                self._input_default_policy=policy
        signal=dict(rule_id=rule,signal_id=identity,observed=value,source_time=source_time,checked_time=checked_time)
        if current:signal['run_id']=current['id']
        if rule=='G01':
            bindings=getattr(self,'_input_identity_guards',None)
            if bindings is None:self._input_identity_guards=bindings={}
            cached=bindings.get(identity)
            if cached is None or cached[0]!=policy['policy_hash'] or cached[1]!=expected_identity:
                cached=(policy['policy_hash'],deepcopy(expected_identity),Guard.bind(policy,dict(expected_identity=expected_identity)))
                bindings[identity]=cached
            decision=cached[2].evaluate(signal,'input_identity')
        else:decision=Guard.evaluate(signal,'input',policy)
        if getattr(self,'event',None) and not realtime:self.event('guard_decision',deepcopy(decision))
        return decision

    def _feedback(self):
        runtime = self.device.runtime
        if runtime is None:raise ValueError('请先连接设备')
        deadline=time.monotonic()+.1
        while True:
            inputs=runtime.control_inputs(include_feedback=False)if self.ros_plane is not None else runtime.control_inputs()
            sample=inputs['calibration_feedback']or{}
            if self.ros_plane is not None:
                if self.ros_inputs is None:raise ValueError('ROS sources must be bound before feedback is consumed')
                try:
                    _,raw=self.ros_inputs.latest('state')
                    sample=dict(raw,source_clock_id=None if self.device.environment=='physical' else raw['source_clock_id'])
                except LookupError:
                    if time.monotonic()>=deadline:raise ValueError('等待已绑定ROS来源的实测反馈')
                    time.sleep(.005);continue
            if (sample.get('command_epoch',inputs['command_epoch'])==inputs['command_epoch']
                    and sample.get('runtime_nonce',inputs['runtime_nonce'])==inputs['runtime_nonce']):
                if (not inputs['feedback_healthy']or not sample or not isinstance(sample.get('sample_wall_ns'),int)):
                    raise ValueError('推理需要健康实测反馈及有效采样时间')
                now=inputs['control_time_s'];stamp=sample['sample_time_s']
                if self.ros_plane is not None:
                    # The RPC projection may precede the selected ROS sample.
                    # Use the declared data clock, never restamp the sample.
                    stamp=sample['source_time_ns']/1e9
                    now=time.monotonic()if self.device.environment=='physical'else self.ros_inputs.latest('state')[0].source_time_ns/1e9
                feedback_decision=self._input_guard('G03',now-stamp,stamp,now,'feedback')
                if feedback_decision['verdict']!='reject'or self.ros_plane is None or time.monotonic()>=deadline:break
            if time.monotonic()>=deadline:raise ValueError('控制权交接后尚无同代反馈；未重标旧反馈，请等待新鲜状态')
            time.sleep(.005)  # Business preparation only; never called by Owner.tick.
        value=sample;healthy=inputs['feedback_healthy']
        epoch,nonce=inputs['command_epoch'],inputs['runtime_nonce'];limits=inputs['joint_limits']
        if (not healthy or not value or not isinstance(value.get('sample_wall_ns'), int)):
            raise ValueError('推理需要健康实测反馈及有效采样时间')
        if feedback_decision['verdict']=='reject':
            raise ValueError(f"G03: 推理需要 {feedback_decision['limit']:g} 秒内的健康实测反馈（当前年龄 {feedback_decision['observed']:.6f} 秒）")
        from .inference_owner import validate_gripper_feedback
        validate_gripper_feedback(tuple(inputs['disabled_grippers']),tuple(self.disabled_grippers),value)
        return now, value, dict(command_epoch=epoch, runtime_nonce=nonce), limits

    def _refresh_observation(self):
        # Preview preparation and the cache thread must not publish out of order.
        with self.observation_lock:return self._refresh_observation_locked()

    def _wait_observation(self):
        from .observation_pairing import PairingPending
        deadline=time.monotonic()+.5
        while True:
            try:return self._refresh_observation()
            except PairingPending:
                if time.monotonic()>=deadline:raise
                time.sleep(.005)  # Observation readiness only; never Owner.tick or a motion retry.

    def _refresh_observation_locked(self):
        with self.lock:prepared,generation = self.prepared,self.generation
        if prepared is None:raise ValueError('尚未绑定当前标定与三相机')
        if not self._binding_current(prepared):raise ValueError('设备、模型会话、标定或相机服务身份已变化，请重新绑定')
        if self.ros_plane is not None:return self._refresh_ros_observation(prepared,generation)
        pairs = {}
        mixed=model_family(self.selected_model_id)=='mixed366'
        for role,camera in CAMERAS.items():
            try:pairs[role]=(self.rgb_source.latest_rgb(camera)if mixed and self.rgb_source is not None else self.cameras.latest_rgbd(camera))
            except (OSError,ConnectionError) as error:
                raise ValueError('无法读取'+{'head':'头部','left':'左腕','right':'右腕'}[role]+
                    '相机 RGB-D：共享相机服务连接中断；请恢复相机服务并重新绑定工作台后再运行。') from error
        # Validate the current source identities before history can request a wait.
        identity_time=time.monotonic()
        for role,camera in CAMERAS.items():
            meta=pairs[role]['metadata']
            if not isinstance(meta.get('capture_instance_id'),str)or not meta['capture_instance_id']:
                raise ValueError('推理相机身份或设备环境已变化')
            expected=dict(serial=prepared['binding']['camera_serials'][role],environment=self.device.environment)
            if prepared.get('capture_instances')is not None:expected['capture_instance_id']=prepared['capture_instances'].get(camera+'_rgb')
            decision=self._input_guard('G01',{key:meta.get(key)for key in expected},identity_time,identity_time,'camera_identity_'+role,expected_identity=expected)
            if decision['verdict']=='reject':
                error=ValueError('相机身份或采集实例已变化，请重新绑定；旧录制和审阅不能继续');error.guard_decision=decision;raise error
        now, feedback, owner, _ = self._feedback()
        pairing=None
        if mixed and self.device.environment=='physical':
            pairs,feedback,pairing=self.observation_pairer.pair(pairs,feedback,owner)
            paired=self._input_guard('G03',now-feedback['sample_time_s'],feedback['sample_time_s'],now,'paired_feedback')
            if paired['verdict']=='reject' and now>feedback['sample_time_s']:
                from .observation_pairing import PairingPending
                raise PairingPending('配对机器人反馈超过原0.1秒新鲜度限制，等待合格的新配对')
            if paired['verdict']=='reject':raise ValueError('G03: 配对机器人反馈超过当前策略新鲜度限制')
        clock = clock_spec(self.device.environment, source_clock_id=feedback.get('source_clock_id'))
        times, images, identities = [], {}, {}
        for role,camera in CAMERAS.items():
            pair = pairs[role];meta = pair['metadata']
            if self.device.environment == 'fake':
                if meta.get('source_clock_id') != clock['source_clock_id'] or meta.get('clock_domain') != 'simulation':
                    raise ValueError('推理图像不属于当前仿真源时钟')
                if type(meta.get('source_time_ns')) is not int:raise ValueError('相机源时间缺失')
                stamp = meta['source_time_ns']/1e9
            else:
                if meta.get('clock_domain','host_wall')!='host_wall' or meta.get('source_clock_id') is not None or meta.get('simulation_only') is True:
                    raise ValueError('真机推理图像不能使用仿真时钟')
                if type(meta.get('host_receive_ns')) is not int:raise ValueError('相机接收时间缺失')
                stamp = feedback['sample_time_s']+(meta['host_receive_ns']-feedback['sample_wall_ns'])/1e9
            age_decision=self._input_guard('G02',now-stamp,stamp,now,'camera_'+role)
            if age_decision['verdict']=='reject':
                raise ValueError(f"G02: 推理图像源时间超前或超过 {age_decision['limit']:g} 秒")
            width, height = pair['width'], pair['height']
            if ((width,height) not in (((640,480),)if mixed else((640,480),(224,224)))
                    or len(pair['rgb']) != width*height*3 or not mixed and len(pair['depth']) != width*height*2):
                raise ValueError('共享相机原始 RGB-D 像素尺寸不匹配')
            image = np.frombuffer(pair['rgb'],dtype=np.uint8).reshape(height,width,3)
            if not mixed:image=letterbox_rgb224(image)
            image.flags.writeable=False
            images[camera+'_rgb'] = image;times.append(stamp)
            identities[camera+'_rgb'] = {key:meta.get(key) for key in ('serial','capture_instance_id','frame_number',
                'depth_frame_number','host_receive_ns','source_time_ns','source_clock_id','physics_step')}
            identities[camera+'_rgb'].update(feedback_sample_time_s=feedback['sample_time_s'],feedback_sample_wall_ns=feedback['sample_wall_ns'])
            if pairing is not None:
                identities[camera+'_rgb'].update(pairing_target_wall_ns=pairing['target_wall_ns'],
                    pairing_error_ns=pairing['camera_error_ns'][role],subscriber_receive_ns=meta.get('subscriber_receive_ns'))
        span_decision=self._input_guard('G04',max(times)-min(times),min(times),now,'camera_span')
        if span_decision['verdict']=='reject':
            raise ValueError(f"G04: 推理三相机帧间跨度超过 {span_decision['limit']:g} 秒")
        captures={name:row['capture_instance_id'] for name,row in identities.items()}
        if prepared.get('capture_instances') is not None and prepared['capture_instances']!=captures:
            raise ValueError('相机采集实例已变化，请重新绑定；旧录制和审阅不能继续')
        value = dict(source_time=min(times), device_state14=device_state(feedback['qpos'],
            disabled_grippers=self.disabled_grippers,right_gripper_device_mm=prepared['profile']['gripper']['max_opening_mm']), images=images,
            frame_ids=identities, feedback_sample_time_s=feedback['sample_time_s'], feedback_sample_wall_ns=feedback['sample_wall_ns'])
        support = None
        if self.support_provider:
            try:support = self.support_provider(now, deepcopy(feedback))
            except Exception as error:support = {'error':str(error)}
        with self.lock:
            if self.prepared is prepared and self.generation==generation and self._binding_current(prepared):
                prepared['capture_instances']=captures
                self.cache = dict(observation=value, owner=owner, binding_digest=prepared['binding_digest'], clock=clock,pairing=pairing)
                self.support_cache, self.cache_error = support, None
        runtime=self.device.runtime
        if getattr(runtime,'remote_control_owner',False):runtime.push_observation(value,support,owner['command_epoch'])

    def _bind_ros_sources(self,binding):
        from .ros_data import RosNode,SourceDirectory
        from .ros_inputs import RosInputs
        from .observation_pairing import TimestampPairer
        runtime=self.device.runtime
        if self.ros_worker is not None:self.ros_worker.close();self.ros_worker=None
        if self.ros_node is not None:self.ros_node.close();self.ros_node=None
        plane=self.ros_plane
        node=RosNode('infer_input_'+self.instance_id[:12],namespace=plane['namespace'],
            domain_id=plane['domain_id'],localhost_only=False)
        try:
            directory=SourceDirectory(node,self.device.id)
            expected={'state':dict(source_instance_id=runtime.nonce,kind='state')}
            expected.update({camera:dict(kind='image')for camera in CAMERAS.values()})
            rows=directory.resolve(expected)
            sources={row['stream_id']:row for row in rows}
            for role,camera in CAMERAS.items():
                if sources[camera]['metadata'].get('serial')!=binding['camera_serials'][role]:
                    raise ValueError('ROS camera serial differs from the installed binding')
            inputs=RosInputs(node,rows)
            publisher=node.publisher('observations',node.types.Observation)
            control=directory.resolve({'control':dict(source_instance_id=runtime.nonce,kind='event')})[0]
            publishers=node.node.get_publishers_info_by_topic(control['topic'])
            if len(publishers)!=1:raise ValueError('one original control event publisher required')
            control_gid=bytes(publishers[0].endpoint_gid)
            def control_event(message):
                try:
                    writers=node.node.get_publishers_info_by_topic(control['topic'])
                    if (len(writers)!=1 or bytes(writers[0].endpoint_gid)!=control_gid or any(getattr(message.reference,key)!=control[key]
                            for key in ('stream_id','source_instance_id','source_clock_id'))):
                        raise ValueError('bound control event source changed; rebind required')
                    self._ros_control_event(message)
                except Exception as error:
                    with self.lock:run=self.current
                    if run is not None and run.get('stall_recovery')is not None:
                        run['stall_recovery'].reset('control_source_error')
                    node.errors.append(str(error))
            node.subscriptions.append(node.node.create_subscription(node.types.DataEvent,control['topic'],control_event,node.qos(depth=128)))
        except BaseException:node.close();raise
        self.ros_node,self.ros_directory,self.ros_inputs=node,directory,inputs
        self.ros_observations=publisher;self.observation_pairer=TimestampPairer()

    def _ros_control_event(self,message):
        from .ros_data import json_value
        with self.lock:run=self.current
        if (run is None or message.run_id!=run['id']
                or message.reference.source_instance_id!=run['owner'].runtime_nonce):return
        name=message.event[len('inference_'):]if message.event.startswith('inference_')else 'other_control_event'
        recovery=run.get('stall_recovery')
        # Raw windows/feedback belong to the independent recorder. Decoding
        # unused payloads on the image/state executor delays fresh inputs.
        needed=name in METRIC_EVENTS or recovery is not None and name in('mode_transition','normal_model_segment_started','feedback')
        if not needed and recovery is None:return
        value=json_value(message.payload_json)if needed else {}
        if recovery is not None:recovery.event(name,value,message.reference.sequence)
        if name in METRIC_EVENTS:self._metric_event(run,name,value)

    def _prepare_ros_model(self):
        from .ros_inference import RosModelWorker
        prepared,client,adapter=self.prepared,self.client,self.adapter
        binding=prepared.get('action_chunk_binding')
        if binding is None:raise ValueError('ROS model requests require the pinned action chunk binding')
        def intervention(request):
            with self.lock:
                run=self.current
                recovery=run.get('stall_recovery')if run is not None and run['id']==request.run_id else None
            return recovery.choose(request.seed)if recovery is not None else None
        def predict(request,prepared_input):
            if not self._binding_current(prepared):raise ValueError('ROS model binding retired')
            started=time.monotonic()
            try:
                return client.infer_chunk(request,epoch=request['epoch'],clock_domain=request['clock_domain'],
                    valid_until_source_time=request['valid_until_source_time'],prepared_input=prepared_input)
            finally:
                with self.lock:
                    run=self.current
                    if run is not None and run['id']==request['run_id']:
                        run['metrics']['returned_predictions']+=1
                        run['metrics']['last_model_wall_s']=time.monotonic()-started
        self.ros_worker=RosModelWorker(self.ros_node,self.ros_inputs,adapter.model_input,predict,binding=binding,
            stall_recovery=intervention,
            clock=lambda:round(self.device.runtime.owner.control_time()*1e9),
            clock_id=self.ros_inputs.sources['state']['source_clock_id'],
            state_to_device=lambda raw:device_state(raw['qpos'],disabled_grippers=self.disabled_grippers,
                right_gripper_device_mm=prepared['profile']['gripper']['max_opening_mm']))
        self.ros_worker.source_descriptors(self.device.id)

    def _ros_capture_sources(self):
        from .ros_recording import capture_source
        control=self.ros_directory.resolve({'control':dict(source_instance_id=self.device.runtime.nonce)})
        sources=[*self.ros_inputs.sources.values(),*control,*self.ros_worker.source_descriptors(self.device.id)]
        pedal=getattr(self,'operator_pedal_source',None)
        if pedal is not None:sources.append(pedal)
        return [capture_source(row,namespace=self.ros_plane['namespace'],domain_id=self.ros_plane['domain_id'])for row in sources]

    def _refresh_ros_observation(self,prepared,generation):
        from .ros_inputs import referenced_observation,observation_times
        from .observation_pairing import PairingPending
        from .inference_owner import validate_gripper_feedback
        frame_streams={camera+'_rgb':camera for camera in CAMERAS.values()}
        frames,state,stamp,pairing=self.ros_inputs.paired(frame_streams,self.observation_pairer)
        feedback=self.ros_inputs.resolve(state)
        latest,raw=self.ros_inputs.latest('state')
        runtime=self.device.runtime
        # The canonical state already carries the current Owner and epoch.
        # A per-selection control RPC queues behind page/recording status reads
        # and can stop fresh observations despite healthy incoming ROS samples.
        if runtime is None or raw.get('runtime_nonce')!=runtime.nonce:
            raise ValueError('ROS feedback belongs to another bound Owner')
        owner={key:raw[key]for key in ('runtime_nonce','command_epoch')}
        if any(feedback.get(key)!=value for key,value in owner.items()):
            raise ValueError('paired ROS state belongs to a retired Owner epoch')
        validate_gripper_feedback(tuple(prepared['disabled_grippers']),tuple(self.disabled_grippers),feedback)
        message=self.ros_inputs.observation(frames,state,run_id='',epoch=owner['command_epoch'],
            model_io_digest=self.client.conversion_id,source_time_ns=stamp,pairing=pairing)
        times=observation_times(message,feedback)
        # Check original stamps against the live source clock, never restamp.
        now=(time.monotonic()if self.device.environment=='physical'else
             latest.source_time_ns/1e9)
        for rule,age,identity in [('G02',now-stamp/1e9,'ros_observation'),
                ('G03',now-state.source_time_ns/1e9,'ros_paired_feedback'),
                ('G04',(max(times)-min(times))/1e9,'ros_camera_span')]:
            decision=self._input_guard(rule,age,stamp/1e9,now,identity)
            if decision['verdict']=='reject':
                raise PairingPending(f"{rule}: waiting for a fresh ROS selection (observed={decision['observed']}, limit={decision['limit']})")
        value=referenced_observation(message,self.ros_inputs.sources,feedback,state_to_device=lambda raw:device_state(
            raw['qpos'],disabled_grippers=self.disabled_grippers,right_gripper_device_mm=prepared['profile']['gripper']['max_opening_mm']))
        value.update(feedback_sample_time_s=state.source_time_ns/1e9,feedback_sample_wall_ns=feedback['sample_wall_ns'])
        captures={name:row['capture_instance_id']for name,row in value['frame_ids'].items()}
        if prepared.get('capture_instances')is not None and captures!=prepared['capture_instances']:
            raise ValueError('ROS camera source changed; explicit rebind required')
        clock=clock_spec(self.device.environment,source_clock_id=None if self.device.environment=='physical' else state.source_clock_id)
        with self.lock:
            if self.prepared is prepared and self.generation==generation:
                prepared['capture_instances']=captures
                self.cache=dict(observation=value,owner=owner,binding_digest=prepared['binding_digest'],clock=clock,pairing=pairing)
                self.support_cache,self.cache_error=None,None
                run=self.current
                if (self.normal_modes is not None and self.ros_worker is not None and run is not None
                        and run['ended_wall']is None and run['owner'].runtime_nonce==owner['runtime_nonce']):
                    # Publish only after the same-owner, fresh state has advanced
                    # the worker binding; a request cannot authorize its own epoch.
                    self.ros_worker.bind(run['id'],owner['command_epoch'])
                self.ros_observations.publish(message)

    def _cache_loop(self):
        while not self.cache_stop.is_set():
            try:self._refresh_observation()
            except Exception as error:
                with self.lock:self.cache_error=str(error);self.cache=None;self.support_cache=None
            self.cache_stop.wait(.02)

    def _start_cache(self):
        with self.lock:
            if not self.cache_paused and self.cache_thread is None:
                self.cache_stop.clear()
                self.cache_thread=threading.Thread(target=self._cache_loop,daemon=True,name='inference-observations')
                self.cache_thread.start()

    def pause_observations(self):
        with self.lock:
            self.cache_paused=True;self.cache_stop.set();thread=self.cache_thread
        if thread is not None:thread.join(3)
        if thread is not None and thread.is_alive():raise ValueError('观测缓存仍有在途调用，维护保持冻结')
        with self.lock:
            if self.cache_thread is thread:self.cache_thread=None

    def resume_observations(self):
        with self.lock:
            thread=self.cache_thread
            if thread is not None and thread.is_alive():raise ValueError('观测缓存仍未退出，不能结束维护')
            self.cache_thread=None;self.cache_paused=False
            prepared=self.prepared is not None
        if prepared:self._start_cache()

    def _observe(self, now, feedback, binding_digest, capture_instances=None,*,owner_identity=None):
        # Memory-only consumer called on Owner.tick; IPC and image conversion live above.
        with self.lock:cached=self.cache
        runtime=self.device.runtime
        expected_owner=(owner_identity if owner_identity is not None else
            dict(command_epoch=runtime.owner.epoch,runtime_nonce=runtime.owner.runtime_nonce)if runtime is not None else None)
        if (not cached or runtime is None or cached['binding_digest'] != binding_digest
                or cached['owner'] != expected_owner
                or cached['clock']['source_clock_id'] != feedback.get('source_clock_id')
                or cached['owner']['runtime_nonce'] != feedback.get('runtime_nonce',cached['owner']['runtime_nonce'])
                or cached['owner']['command_epoch'] != feedback.get('command_epoch',cached['owner']['command_epoch'])):
            return None
        value=cached['observation']
        for rule,stamp,identity in [('G02',value['source_time'],'cached_observation'),
                ('G03',value['feedback_sample_time_s'],'cached_feedback'),('G03',feedback['sample_time_s'],'current_feedback')]:
            if self._input_guard(rule,now-stamp,stamp,now,identity,realtime=True)['verdict']=='reject':return None
        if capture_instances is not None and any(value['frame_ids'][CAMERAS[role]+'_rgb']['capture_instance_id']!=identity for role,identity in capture_instances.items()):
            return None
        return {key:value[key] for key in ('source_time','device_state14','images','frame_ids','ros_observation') if key in value}

    def _support(self, now, feedback):
        with self.lock:return deepcopy(self.support_cache)

    def _prepare(self, seed):
        with self.lock:self.seed=seed
        return self.bind_device()

    def _preview(self, seed, duration_s=120,reference_feedforward_sha256=None,*,wam_stall_recovery=False):
        duration_s=episode_duration(duration_s)
        if type(wam_stall_recovery)is not bool:raise ValueError('WAM trick requires an explicit boolean')
        if wam_stall_recovery and not self.wam_stall_recovery_available:raise ValueError('当前部署未开放 Trick，请使用普通推理')
        if wam_stall_recovery and (model_family(self.selected_model_id)!='wam' or self.ros_plane is None
                or self.normal_modes is None or self.reference_planning is None
                or self.recording_backend!='capture_session_v1'):
            raise ValueError('WAM trick requires the WAM ROS complete-segment workflow and resident recorder')
        if self.ros_plane is not None and self.recording_backend=='capture_session_v1':
            from .recording_process import require_recording_protocol
            require_recording_protocol()
        if self.finite_chunk_count is not None and duration_s!=.5:raise ValueError('每轮模型源轨迹固定0.5秒15步')
        with self.lock:
            if self.snapshot()['busy']:raise ValueError('当前推理或录制仍未结束')
            self._sync_control_policy()
            prepared,client,adapter=self.prepared,self.client,self.adapter
            generation=self.generation
        if prepared is None or client is None:raise ValueError('请先加载并预热模型')
        if not self._binding_current(prepared):raise ValueError('设备、标定或相机服务身份已变化，请重新绑定')
        if self._binding() != (prepared['profile'],prepared['binding']):raise ValueError('标定或相机绑定已变化，请重新准备模型接入')
        now,feedback,owner_identity,limits=self._feedback()
        if limits != prepared['limits']:raise ValueError('设备限位已变化，请重新准备模型接入')
        if self.reference is not None:
            if (duration_s!=2 or self.reference['model_id']!=self.selected_model_id
                    or self.reference['calibration_profile_digest']!=prepared['binding']['calibration_profile_digest']):
                raise ValueError('固定研究轨迹仅支持其绑定的模型、标定和2秒时长')
            expected=self.reference['device_targets14'][0]
            if max(abs(a-b)for a,b in zip(feedback['qpos'][:6],expected[:6]))>DEFAULT_JOINT_TOLERANCE_RAD:
                raise ValueError(f'固定研究轨迹起点偏差超过{DEFAULT_JOINT_TOLERANCE_RAD:g}rad，请先按已审阅的人工轨迹对齐')
        if self.action_chunks and not getattr(self.device.runtime,'remote_control_owner',False):
            raise ValueError('action chunks require the context-aware remote Owner execution path')
        deadline=time.monotonic()+.5
        while True:
            observation=self._observe(now,feedback,prepared['binding_digest'],owner_identity=owner_identity)
            if observation is not None:break
            if time.monotonic()>=deadline:raise ValueError(self.cache_error or '等待新鲜三相机观测')
            # Binding/status reads can outlive the observation selected just
            # before them. Keep this one preview intent while fresh input arrives.
            time.sleep(.005)
            from .observation_pairing import PairingPending
            try:self._refresh_observation()
            except PairingPending:pass
            now,feedback,owner_identity,limits=self._feedback()
            if limits!=prepared['limits']:raise ValueError('设备限位已变化，请重新准备模型接入')
        captures={role:observation['frame_ids'][camera+'_rgb']['capture_instance_id'] for role,camera in CAMERAS.items()}
        source_clock=clock_spec(self.device.environment,source_clock_id=feedback.get('source_clock_id'))
        config=dict(model_id=self.selected_model_id,model_identity=deepcopy(self.model_identity),
            model_checkpoint_sha256=self.selected_spec['pins']['checkpoint'],
            conversion_identity=prepared['conversion'],conversion_id=prepared['conversion_id'],binding=prepared['binding'],
            calibration_profile_digest=prepared['binding']['calibration_profile_digest'],camera_serials=prepared['binding']['camera_serials'],
            camera_capture_instances=captures,camera_service_instance_id=getattr(self.cameras,'descriptor',{}).get('instance_id'),clock=source_clock,
            duration_s=duration_s,control_hz=30,execute_steps=self.execute_steps,seed=seed,collision_check=False,position_projection=True,
            motion_audit='record_only',maximum_observation_age_s=2.5 if model_family(self.selected_model_id)=='wam' else .5,recording_enabled=self.recording_enabled,
            velocity_limits=[2.]*6+[None]+[2.]*6+[None],acceleration_limits=[5.]*6+[None]+[5.]*6+[None])
        config['disabled_grippers']=list(self.disabled_grippers)
        if self.recording_backend is not None:config['recording_backend']=self.recording_backend
        if self.ros_plane is not None:config['data_plane']=deepcopy(self.ros_plane)
        if self.reference_planning is not None:config['reference_planning']=deepcopy(self.reference_planning)
        if self.finite_chunk_count is not None:config['finite_chunk_count']=self.finite_chunk_count
        if self.action_chunks:config['action_chunk_binding']=deepcopy(prepared['action_chunk_binding'])
        if self.normal_modes is not None:
            config.update(normal_modes=deepcopy(self.normal_modes),normal_initial_mode=self.normal_initial_mode)
            if self.reference_planning is not None:config['reference_supply_timeout_s']=.5
        policy_parameters=getattr(self,'_control_policy_parameters',{})
        if 'observation' in policy_parameters:
            origin=(getattr(self,'_input_policy_snapshot',{})or{}).get('origins',{}).get('observation.maximum_age_s')
            if origin!='product_default':config['maximum_observation_age_s']=policy_parameters['observation']['maximum_age_s']
            for key in ('state_max_age_s','max_camera_span_s'):config[key]=policy_parameters['observation'][key]
        if 'model' in policy_parameters:config['reference_supply_timeout_s']=policy_parameters['model']['reference_supply_timeout_s']
        for key in ('prepare_drift_joint_rad', 'prepare_drift_jaw_mm'):
            if key in policy_parameters.get('execution', {}):config[key] = policy_parameters['execution'][key]
        if self.normal_finish_policy is not None:
            config.update(normal_finish_kind=self.normal_finish_kind,normal_finish_policy=deepcopy(self.normal_finish_policy))
        if model_family(self.selected_model_id)in('mixed366','wam'):config['task']=self.task
        shared_profile=getattr(self.device.runtime.owner,'shared_target_profile',None)
        if shared_profile:
            config.update(target_interpreter_profile=shared_profile,target_execution='shared_reference_windows')
            if reference_feedforward_sha256 is not None and shared_profile!='normal_X5':
                raise ValueError('该冻结前馈仅资格于normal_X5；当前共同控制配置不同，需重新验证后才能选择')
        if model_family(self.selected_model_id)=='mixed366'and duration_s==.5 and self.finite_chunk_count is None and self.normal_modes is None:
            config.update(bounded_live_prediction=True,execute_steps=15)
        if self.disabled_grippers:
            mapping=prepared['conversion']['mapping']
            for key in ('disabled_right_gripper_model_m','disabled_right_gripper_device_mm','disabled_right_gripper_assumption'):
                config[key]=mapping[key]
        if self.reference is not None:
            config['reference_trajectory']=deepcopy(self.reference_pin)
            config['trajectory_kind']='frozen_model_reference'
        compiled=None;feedforward=None
        if reference_feedforward_sha256 is not None:
            if not self.reference_integral_feedback:raise ValueError('feedforward requires qualified research feedback profile')
            from .motion_reference import load_feedforward
            feedforward=load_feedforward(self.output/'reference-feedforward',reference_feedforward_sha256,self.reference_pin['sha256'])
            config['reference_feedforward']=dict(sha256=feedforward['sha256'],knots=len(feedforward['phase']),provenance=feedforward['provenance'])
        if self.prepared_reference_execution:
            from arx_env.joint_head_real_motion import feedback_to_command
            from arx_env.joint_head_jaw_mapping import command_sdk
            from .motion_reference import prepare_reference_process
            initial=list(feedback_to_command(feedback['qpos'],limits,environment=self.device.environment))
            if self.disabled_grippers:initial[13]=command_sdk('right',config['disabled_right_gripper_device_mm'])
            compiled=prepare_reference_process(self.reference,initial,limits,source_sha256=self.reference_pin['sha256'])
            config['reference_integral_feedback']=self.reference_integral_feedback
            from .motion_reference import native_holds
            config.update(prepared_reference=deepcopy(compiled.metadata),execution_duration_s=compiled.duration_s+sum(native_holds(config)),
                          trajectory_kind='prepared_native_reference')
        debug_request=self.debug_request
        def new_run(owner,configuration,control_policy=None):
            with self.lock:
                if self.generation!=generation:raise ValueError('本轮推理准备已撤销')
                if not self._binding_current(prepared) or self._model_gate()['ready'] is not True:
                    raise ValueError('推理依赖已变化，本轮执行已撤销')
                run=_Run(id=owner.execution_instance_id,directory=self.output/('inference-'+owner.execution_instance_id),
                    config=deepcopy(configuration),prepared=prepared,owner=owner,runner=None,recorder=None,
                    recording_error=None,receipt=None,compiled_reference=compiled,started_wall=None,ended_wall=None,feedback_count=0,event_bytes=0,
                    metrics=dict(queries=0,predictions=0,returned_predictions=0,accepted_predictions=0,
                        discarded_predictions=0,consecutive_discarded_predictions=0,discard_reasons={},
                        last_discard_reason=None,last_prediction_status=None,last_observation_age_source_s=None,
                        last_model_wall_s=None,last_infer_wall_s=None),
                    audit=dict(counts=dict(position=0,velocity=0,acceleration=0),projection_count=0,unavailable_axes={}))
                self.current=run;self.status='running';self.stage='inference.execute';self.message='已确认本轮推理；等待设备开始。'
                if wam_stall_recovery:
                    from .wam import StallRecovery
                    run['stall_recovery']=StallRecovery(prepared['profile']['gripper']['max_opening_mm'],
                        feedback_max_age_s=configuration['state_max_age_s'])
                if self.ros_worker is not None:self.ros_worker.bind(run['id'],owner.epoch)
                if configuration.get('recording_backend')=='capture_session_v1':
                    if control_policy is None:control_policy=self.device.runtime.execution_calibration['control_policy']
                    run['control_policy']=deepcopy(control_policy);reference=weakref.ref(run)
                    nonce=owner.runtime_nonce
                    def current():
                        value=reference()
                        return (value is not None and self.current is value and self.device.runtime is not None
                            and self.device.runtime.nonce==nonce)
                    def published(capture,run=run):
                        try:self._finish_capture(run,capture)
                        finally:
                            with self.lock:self.pending_captures.pop(run['id'],None)
                    recorder_config=dict(configuration,execution_context=dict(device_id=self.device.id,environment=self.device.environment))
                    if self.ros_plane is not None and configuration.get('normal_modes')and configuration.get('normal_initial_mode')=='HOLD':
                        from .recording_activity import POLICY
                        recorder_config['execution_context']['recording_activity']=POLICY
                    recorder_config['execution_context']['wam_stall_recovery']=dict(enabled=wam_stall_recovery,
                        config=run['stall_recovery'].config if wam_stall_recovery else None)
                    run['recorder']=self.capture_factory(run['id'],run['directory'],recorder_config,control_policy,self.cameras,
                        self.device.environment,self.input_evidence,current,published,
                        **(dict(ros_sources=self._ros_capture_sources,ros_watermarks=self.ros_worker.watermarks)if self.ros_plane is not None else{}))
                    self.pending_captures[run['id']]=run['recorder']
                    run['debug_request']=debug_request
                    if debug_request is not None:
                        debug_request['execution_instance_id']=run['id']
                        self.capture_requests[debug_request['job_id']]=run['recorder']
                        self.artifacts.bind(debug_request,run['id'],configuration)
            return run
        def predict(request,consumer_context=None):
            def infer():
                return client.infer(request) if consumer_context is None else client.infer_chunk(request,**consumer_context)
            with self.lock:current_run=self.current
            capture=current_run.get('recorder') if current_run and current_run['config'].get('recording_backend')=='capture_session_v1' else None
            if capture is not None:capture.model_begin(request['request_id'],request['observation']['source_time'])
            if model_family(self.selected_model_id)!='mixed366' and capture is None:return infer()
            timing=dict(clock='host_monotonic_ns',callback_started_ns=time.monotonic_ns())
            state,images=adapter.model_input(request['observation'])
            timing['input_adapted_ns']=time.monotonic_ns()
            try:self.input_evidence.input(request['request_id'],state,images,dict(request_id=request['request_id'],seed=request['seed'],
                task=config.get('task'),model_id=config['model_id'],conversion=prepared['conversion'],
                model_bundle=getattr(client,'model_bundle',None),history_steps=0 if model_family(self.selected_model_id)in('mixed366','wam') else 60,
                observation={k:v for k,v in request['observation'].items()if k!='images'}))
            except Exception as error:
                if capture is None:raise
                capture._fail(error)
            timing['model_started_ns']=time.monotonic_ns()
            try:result=infer()
            except Exception as error:
                try:
                    future=self.input_evidence.failure(request['request_id'],error)
                    if capture is not None:capture.model_end(request['request_id'],future)
                except Exception as write_error:
                    if capture is not None:capture.model_end(request['request_id'],False);capture._fail(write_error)
                raise
            timing['model_returned_ns']=time.monotonic_ns()
            result=dict(result,audit=dict(result.get('audit')or{},
                workbench_call_wall_s=(timing['model_returned_ns']-timing['model_started_ns'])/1e9))
            try:
                future=self.input_evidence.output(request['request_id'],result,chunk=consumer_context is not None,timing=timing)
                if capture is not None:capture.model_end(request['request_id'],future)
            except Exception as error:
                if capture is None:raise
                capture.model_end(request['request_id'],False);capture._fail(error)
            return result
        def factory(owner,proxy,initial,configuration):
            run=new_run(owner,configuration)
            run_ref=weakref.ref(run)
            def callback(method,*args):
                current=run_ref()
                if current is None:
                    if method=='_metric_event':return
                    raise ValueError('inference run retired before recording callback')
                return getattr(self,method)(current,*args)
            def recording_prepared():
                current=run_ref()
                if current is not None and configuration.get('recording_backend')=='capture_session_v1' and current['recorder'].error:
                    raise ValueError(current['recorder'].error)
                return bool(current is not None and current['recorder'] and current['recorder'].prepared.is_set())
            from .inference_reference import ReferencePredictions
            reference=ReferencePredictions(self.reference,adapter,self.reference_pin) if self.reference is not None else None
            runner=make_inference_runner(owner,proxy,initial,configuration,predict=reference.infer if reference is not None else predict,
                observe=lambda now,fb:self._observe(now,fb,prepared['binding_digest'],configuration['camera_capture_instances']),adapter=adapter,
                support=self._support,event=lambda name,value:callback('_metric_event',name,value),
                record=(lambda name,value:callback('_record_event',name,value)) if configuration['recording_enabled'] or self.recording_backend else None,
                record_prepared=recording_prepared if configuration['recording_enabled'] or self.recording_backend else None,
                preloaded_prediction=reference.infer(dict(request_id='reference-'+self.reference_pin['sha256'])) if reference is not None and compiled is None else None,
                prepared_reference=compiled,feedforward_profile=feedforward)
            run['runner']=runner
            if self.recording_backend:
                runner.control_policy=deepcopy(run['control_policy']);runner.guard_policy=deepcopy(run['control_policy'])
            return runner
        runtime=self.device.runtime
        if getattr(runtime,'remote_control_owner',False):
            config.setdefault('reference_supply_timeout_s',.5)
            from .owner_inference import reference_wire,plain
            from .inference_reference import ReferencePredictions
            reference=ReferencePredictions(self.reference,adapter,self.reference_pin)if self.reference is not None else None
            def created(owner,runner,configuration,control_policy=None):
                run=new_run(owner,configuration,control_policy);run['runner']=runner
                if self.recording_backend:runtime.attach_recording(run['recorder'])
            def current_call(method,*args):
                with self.lock:run=self.current
                if run is None:raise ValueError('inference execution not attached to this business')
                return getattr(self,method)(run,*args)
            runtime.configure_remote_inference(config,dict(profile=prepared['profile'],reference=reference_wire(compiled),
                preloaded_prediction=plain(reference.infer(dict(request_id='reference-'+self.reference_pin['sha256'])))
                    if reference is not None and compiled is None else None,feedforward=None),created=created,
                metric=lambda *args:current_call('_metric_event',*args),record=lambda *args:current_call('_record_event',*args),
                predict=reference.infer if reference is not None else predict,
                **({'predict_chunk':predict} if self.action_chunks else {}))
            self._policy_cache_key=None;self._sync_control_policy()
            # CPU reference preparation may outlive the feedback freshness
            # window. Publish a new observation, never restamp the old one.
            self._wait_observation()
        else:
            runtime.configure_inference(config,factory)
            self._policy_cache_key=None;self._sync_control_policy()
        if callable(getattr(client,'bind_control_policy',None)):
            client.bind_control_policy(getattr(self,'_input_policy_snapshot',None))
        preview=self.device.submit('inference',dict(action='preview_inference',seed=seed,duration_s=duration_s))
        preview.update(workflow_generation=generation,seed=seed,duration_s=duration_s)
        with self.lock:
            if self.generation!=generation:raise ValueError('本轮推理准备已撤销')
            self.seed=seed;self.duration_s=duration_s;self.current=None;self.status='awaiting_review';self.stage='inference.review';self.message='请核对整轮执行摘要并确认一次。'
        summary=(self.selected_spec['label']+f'；本轮 {duration_s:g} 个设备源时间秒，随机种子 '+str(seed)+
            f'。每次预测{WAM_HORIZON if model_family(self.selected_model_id)=="wam" else 60}步、连续执行{config["execute_steps"]}步后重新观测；全轮只确认一次。位置/速度/加速度仅审计记录，'
            '位置越界显式投影并保留原始与实际目标，速度/加速度不改速；不检查碰撞。录像绑定当前三相机与标定。'
            '任务成功仅依据末段稳定托盘支撑；停止仍可随时请求。')
        if shared_profile:
            summary=summary.replace('位置/速度/加速度仅审计记录，','模型原始点的动态审计仅记录，').replace(
                '速度/加速度不改速；','连续参考另受规划速度/加速度约束，实际时长以参考摘要为准；')
        if not self.recording_enabled:summary=summary.replace('录像绑定当前三相机与标定。','本轮录制已关闭，不生成 RGB-D 数据或过程视频；保留控制事件日志。')
        if config.get('bounded_live_prediction'):
            summary=('实时'+self.task+'推理；先保持并取得新鲜模型预测，再执行0.5秒并Stop撤销未来目标。'
                '首个预测点对应观测后1/30秒；双臂与夹爪按训练原始坐标转换。'
                '沿用共同准备层速度/加速度约束，若重定时则记录时间比例；0.5秒指实际控制区间。')
        if self.reference_planning is not None:
            policy=self.reference_planning
            summary+=f' 规划峰值：关节速度{policy["velocity_limit_rad_s"]:g}rad/s、加速度{policy["acceleration_limit_rad_s2"]:g}rad/s²、夹爪{policy["jaw_velocity_limit_mm_s"]:g}mm/s；起止导数约束作用于完整执行片段。'
        if self.finite_chunk_count is not None:
            summary=(f'连续{self.finite_chunk_count}轮；每轮重新观测和推理，完整执行15步、源时长0.5秒，允许减速后每轮实际执行不超过{self.reference_planning["max_execution_duration_s"]:g}秒；不截断或丢点。'
                '轮间零速保持等待，推理等待另行计时；全部完成后停止，任一异常即停且不自动重试。'+summary)
        if self.reference is not None:summary='固定2秒研究轨迹回放；执行期间不重新推理。'+summary
        if compiled is not None:
            initial_hold,terminal_hold=native_holds(config)
            summary=f'原始素材2秒；连续参考实际执行约 {config["execution_duration_s"]:.3f} 秒，含起始保持{initial_hold:g}秒、末端保持{terminal_hold:g}秒。'+summary
        if self.reference_integral_feedback:summary+='本轮仅启用有界低频积分前馈；位置参考与原kp/kd不变，Stop清除附加前馈。'
        if shared_profile:
            summary=summary.replace('位置/速度/加速度仅审计记录，','原预测的位置/速度/加速度保留审计，')
            summary=summary.replace('速度/加速度不改速；','实际执行由共同准备层约束速度/加速度，并单独记录实际时长；')
            summary=summary.replace('位置参考与原kp/kd不变，','位置参考与共同控制策略绑定，')
            summary+='所有运动经共同目标解释器；控制配置='+shared_profile+'。在线预测片段允许显式重定时，不能在剩余时长内完整执行的片段不下发，保持前段零速末端。'
        if feedforward is not None:summary+='复用同源实测前馈候选 '+feedforward['sha256'][:12]+'；候选身份及实际前馈随本轮记录。'
        if self.disabled_grippers:summary+='右臂保持本轮起始关节；右夹爪禁用，模型槽13使用标定最大开度假设，非实测，不向该电机输出。'
        return dict(preview=preview,summary=summary,message='本轮推理已准备，等待一次执行确认')

    def _metric_event(self, run, name, value):
        with self.lock:
            metrics=run['metrics']
            if name in ('prediction_received','prediction_discarded'):
                metrics['returned_predictions']+=1
                metrics['last_infer_wall_s']=metrics['last_model_wall_s']=value.get('model_wall_s')
                metrics['last_observation_age_source_s']=value.get('observation_age_source_s')
                metrics['last_prediction_status']='accepted' if name=='prediction_received' else 'discarded'
                run['audit']['last_worker_timing']=deepcopy((value.get('worker_audit') or {}).get('timing',{}))
            if name=='prediction_requested':run['metrics']['queries']+=1
            elif name=='bounded_prediction_result'and (run.get('config',{}).get('finite_chunk_count')
                    or run.get('config',{}).get('normal_modes')and run.get('config',{}).get('reference_planning')):
                if self.ros_plane is None:metrics['returned_predictions']+=1
                metrics['last_observation_age_source_s']=value.get('observation_age_s')
                metrics['last_prediction_status']='prepared_not_admitted'
                run['audit']['last_preparation_timing']=deepcopy(value.get('preparation'))
            elif name in ('finite_chunk_prepared','bounded_prediction_prepared'):
                audit=value.get('worker_audit')or{}
                metrics['last_infer_wall_s']=audit.get('wall_s')
                metrics['last_model_wall_s']=audit.get('workbench_call_wall_s')
                run['audit']['last_prepared_reference']=deepcopy(value.get('reference'))
            elif name in ('finite_chunk_started','normal_model_segment_started'):
                metrics['predictions']+=1;metrics['accepted_predictions']+=1
                metrics['consecutive_discarded_predictions']=0
                metrics['last_prediction_status']='accepted'
            elif name=='prediction_received':
                metrics['predictions']+=1;metrics['accepted_predictions']+=1
                metrics['consecutive_discarded_predictions']=0
                audit=value.get('audit') or {}
                for problem in audit.get('problems',[]):
                    kind=problem.get('kind')
                    if kind in run['audit']['counts']:run['audit']['counts'][kind]+=1
                run['audit']['unavailable_axes']=deepcopy(audit.get('unavailable_axes',{}))
                run['audit']['projection_count']+=sum(len(row.get('projections',[])) for row in value.get('conversions',[]))
            elif name=='prediction_discarded':
                metrics['discarded_predictions']+=1;metrics['consecutive_discarded_predictions']+=1
                reason=value.get('reason') or 'unknown'
                metrics['last_discard_reason']=reason
                metrics['discard_reasons'][reason]=metrics['discard_reasons'].get(reason,0)+1
            elif name=='episode_started':run['started_wall']=time.monotonic()
            elif name in ('episode_completed','episode_stopped')and not run['config'].get('normal_modes'):
                run['ended_wall']=time.monotonic()

    def _record_event(self, run, name, value):
        """Dedicated InferenceRunner data thread only; never called from Owner.tick."""
        if name=='finish':
            with self.lock:run['ended_wall']=time.monotonic()
        if run.get('config',{}).get('recording_backend')=='capture_session_v1':
            run['recorder'].record(name,value)
            failed=getattr(run.get('runner'),'_recording_failure',None)
            if run['recorder'].error and callable(failed):failed(run['recorder'].error)
            return
        from robot_workbench.recording_process import ProcessRecordingSession
        if name=='prepare':
            run['directory'].mkdir(parents=True,exist_ok=False)
            clock=value['clock'] or (clock_spec('physical') if self.device.environment=='physical' else None)
            if clock is None or clock_spec(**clock)!=run['config']['clock']:raise ValueError('推理录制源时钟与审批不匹配')
            prepared=run['prepared'];owner=deepcopy(value['execution_owner'])
            source=dict(schema_version='policy_execution_source.v1',environment=self.device.environment,
                robot_id=self.device.id,camera_serials=prepared['binding']['camera_serials'],calibration_profile=prepared['profile'],
                clock=clock,execution_instance_id=run['id'],runtime_nonce=owner['runtime_nonce'],
                command_epoch=owner['command_epoch'],duration_s=run['config'].get('execution_duration_s',run['config']['duration_s']),duration_tolerance_s=POLICY_DURATION_TOLERANCE_S)
            if run['config'].get('target_execution'):
                source['target_execution']=run['config']['target_execution']
            if run.get('compiled_reference')is not None:
                compiled=run['compiled_reference'];source['control_reference']=deepcopy(compiled.metadata)
                source['source_material_duration_s']=run['config']['duration_s']
                np.savez(run['directory']/'prepared-reference.npz',breaks=compiled.breaks,
                    coefficients=compiled.coefficients,jaws=compiled.jaws)
                self._write_json(run['directory']/'prepared-reference.json',compiled.metadata)
            run['source']=source;run['execution_owner']=owner
            self._write_json(run['directory']/'configuration.json',run['config'])
            deadline=value['preparation_deadline_monotonic']
            device_owner=run['owner'];candidate_ref=None;run_ref=weakref.ref(run)
            def current():
                runtime=self.device.runtime
                current_run=run_ref();candidate=candidate_ref()if candidate_ref is not None else None
                return (current_run is not None and runtime is not None and runtime.owner is device_owner and runtime.holder=='inference'
                    and device_owner.runner is current_run['runner'] and device_owner.epoch==owner['command_epoch']
                    and device_owner.runtime_nonce==owner['runtime_nonce']
                    and getattr(device_owner,'execution_instance_id',None)==current_run['id']
                    and current_run['runner'].status not in ('STOPPED','FAULT')
                    and (candidate is not None and candidate.prepared.is_set() or time.monotonic()<deadline))
            def recording_fault(error):
                current_run=run_ref()
                if current_run is not None:self._recording_fault(current_run,error)
            candidate=ProcessRecordingSession(run['directory']/'raw',session_kind='streaming',source=source,binding=prepared['binding'],
                camera_channels=self.cameras.channels,execution_owner=owner,
                preparation_active=current,preparation_deadline_monotonic=deadline,
                on_fault=recording_fault)
            candidate_ref=weakref.ref(candidate)
            # The camera producer admits a sampled lease through owner.recording.
            # Publish the same session before record_prepared can let motion begin.
            with device_owner.lock:
                previous=getattr(device_owner,'recording',None)
                prior_done=previous is None or (getattr(previous,'done',None) is not None and previous.done.is_set()
                    and getattr(previous,'thread',None) is not None and not previous.thread.is_alive())
                attached=current() and prior_done and candidate.execution_instance_id==run['id']
                if attached:device_owner.recording=candidate
                run['recorder']=candidate
            if not attached:
                candidate.finish('stopped','inference preparation revoked before Owner attachment')
                raise ValueError('推理录制准备已撤销或另一个录制仍在收尾，未替换当前 Owner 录制')
        elif name=='command':
            recorder=run['recorder']
            if recorder is None:raise ValueError('本轮录制租约未建立')
            raw=deepcopy(value['feedback_at_send'])
            fields={key:value[key] for key in ('tick','send_monotonic_s','send_wall_ns','owner_pid','command_epoch','runtime_nonce')}
            fields.update(target=physical14(value['final_device14']),feedback_at_send=raw,
                clock_domain=run['source']['clock']['clock_domain'],source_clock_id=run['source']['clock']['source_clock_id'])
            if self.device.environment=='fake':fields.update(send_source_time_ns=value['send_source_time_ns'],physics_step=None)
            if 'native_reference'in value:fields.update(command_kind='native_future_window',native_reference=deepcopy(value['native_reference']))
            recorder.append_command(**fields)
        elif name=='native_window':
            recorder=run['recorder']
            if recorder is None:raise ValueError('本轮录制租约未建立')
            recorder.submit('native_window',**value)
        elif name=='feedback':
            raw=deepcopy(value['raw']);recorder=run['recorder']
            if recorder is None:raise ValueError('本轮录制租约未建立')
            clock_fields={}
            if self.device.environment=='physical':
                clock=clock_spec(**run['source']['clock'])
                # Physical SDK samples need not repeat the clock declaration.
                # Validate the untouched sample, then supply the sealed physical
                # domain explicitly so the shared writer cannot insert None.
                sample_stamp(raw,clock,wall_key='sample_wall_ns')
                clock_fields=dict(clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'])
            recorder.append_feedback(tick=run['feedback_count'],state=physical14(value['device_state14']),
                sample_monotonic_s=raw['sample_time_s'],sample_wall_ns=raw['sample_wall_ns'],raw=raw,
                **run['execution_owner'],**clock_fields)
            run['feedback_count']+=1
        elif name=='finish':
            self._finish_recording(run,value);return
        if name not in ('command','feedback','native_window'):
            path=run['directory']/'policy-events.jsonl'
            blob=(json.dumps(dict(event=name,**value),allow_nan=False,separators=(',',':'))+'\n').encode()
            if len(blob)>512*1024 or run['event_bytes']+len(blob)>128*1024**2:raise ValueError('policy event evidence exceeded bounded size')
            with path.open('ab') as stream:stream.write(blob)
            run['event_bytes']+=len(blob)
            if self.event:self.event(name,deepcopy(value))

    def _recording_fault(self, run, error):
        with self.lock:run['recording_error']=str(error)

    def _finish_capture(self,run,capture):
        manifest=capture.manifest or {};state=capture.snapshot()
        if getattr(self,'ros_plane',None) is not None:
            path=run['directory']/'receipt.json'
            if not path.is_file():raise ValueError('resident recorder Run receipt is missing')
            receipt=json.loads(path.read_text())
            with self.lock:
                run['receipt']=dict(receipt,artifact_status=state.get('recording_outcome')or'incomplete')
                run['recording_error']=state.get('error')
            self.capture_results.notify()
            return
        error=capture.error or manifest.get('playback_error')
        request=run.get('debug_request') or {}
        receipt=dict(schema='workbench.inference_debug.v1',job_id=request.get('job_id',run['id']),execution_instance_id=run['id'],configuration=run['config'],
            control_policy=run['control_policy'],controller=capture.controller_result,recording=state,
            artifact_status=state.get('recording_outcome') or 'incomplete',recording_error=error,
            status=(capture.controller_result or {}).get('status','unknown'),error=error,
            directory=str(run['directory']),model_requests=list(capture.models),missing_files=list(manifest.get('issues',[])),
            trace=dict(event_counts={key:value.get('committed',0) for key,value in manifest.get('streams',{}).items()}))
        run['directory'].mkdir(parents=True,exist_ok=True)
        session=getattr(capture,'session',None)
        service_receipt=session is not None and 'control_policy'in(session.binding or{})
        if not service_receipt:self._write_json(run['directory']/'receipt.json',receipt)
        path=run['directory']/'debug-receipt.json';self._write_json(path,receipt)
        with self.lock:
            run['receipt']=receipt;run['recording_error']=error
        self._debug_finished(receipt,path)
        if service_receipt:
            self.capture_results.notify();return
        rows=[]
        for stream,video in manifest.get('playback',{}).get('streams',{}).items():
            identity=run['id']+'-'+stream
            url='/api/result-media?'+urlencode(dict(environment=self.device.environment,business='inference',result_id=identity))
            result=dict(id=identity,run_id=run['id'],kind='replay',label='Infer · '+stream,status='valid',
                media=dict(id=identity,run_id=run['id'],kind='replay',role='source',camera=stream,ready=True,mime_type='video/mp4',url=url),
                details=dict(frame_count=video['frame_count'],duration_s=video['duration_s'],source_time='original frame intervals'))
            rows.append((result,capture.root/video['path']))
        if rows:self.history.append(rows)

    def _debug_finished(self,receipt,path):
        identity='debug-'+receipt['job_id']
        url='/api/result-media?'+urlencode(dict(environment=self.device.environment,business='inference',result_id=identity))
        result=dict(id=identity,run_id=receipt['execution_instance_id'] or receipt['job_id'],kind='inference_debug',
            label='Infer 调试录制 · '+receipt['status'],status=receipt['status'],download_url=url,
            details=dict(controller_outcome=receipt['status'],reason=receipt.get('error'),
                artifact_status=receipt['artifact_status'],directory=receipt['directory'],
                model_requests=len(receipt['model_requests']),event_counts=receipt['trace'].get('event_counts'),
                missing_files=receipt['missing_files']))
        self.history.append([(result,path)])

    def _finish_recording(self, run, value):
        from robot_workbench.recording_process import export_playback_process
        recorder=run['recorder'];exported=None;error=run['recording_error'] or value.get('recording_error')
        if recorder is not None:
            recorder.finish(value['status'],value.get('reason'))
            if not recorder.done.wait(600):error=error or '录制封存超过 600 秒未完成'
            thread=getattr(recorder,'thread',None)
            if thread is not None and recorder.done.is_set():thread.join(2)
            if thread is not None and thread.is_alive():error=error or '录制写入线程仍未退出'
            status=recorder.snapshot();error=error or status.get('error') or status.get('seal_error')
            if recorder.manifest:
                if recorder.manifest.get('capture_status')!='complete':
                    error=error or '原始录制校验未通过：'+'; '.join(recorder.manifest.get('validation',{}).get('problems',[]))
                if recorder.manifest.get('command_count',0)>0:
                    try:exported=export_playback_process(recorder.root,run['directory']/'videos',environment=self.device.environment)
                    except Exception as exc:error=error or '视频导出失败：'+str(exc)
        else:error=error or '录制准备未完成'
        receipt=dict(schema='inference_run.v1',execution_instance_id=run['id'],configuration=run['config'],
            controller=dict(status=value['status'],reason=value.get('reason')),inference=value.get('inference'),
            recording=recorder.snapshot() if recorder is not None else None,recording_error=error,playback=exported)
        run['directory'].mkdir(parents=True,exist_ok=True)
        self._write_json(run['directory']/'receipt.json',receipt)
        rows=[]
        for filename in (exported or {}).get('files',{}):
            if not filename.endswith('.mp4'):continue
            camera=filename.removeprefix('source-').removesuffix('.mp4');ident=run['id']+'-'+camera
            url='/api/result-media?'+urlencode(dict(environment=self.device.environment,business='inference',result_id=ident))
            result=dict(id=ident,kind='recording',label='推理录像 · '+camera,run_id=run['id'],status='invalid' if error else 'valid',
                details=dict(task_outcome=(value.get('inference') or {}).get('outcome'),recording_error=error,
                    duration_coverage=(recorder.manifest or {}).get('duration_coverage'),
                    actual_duration_s=(recorder.manifest or {}).get('actual_duration_s'),
                    calibration_profile_digest=run['config']['calibration_profile_digest'],conversion_id=run['config']['conversion_id']),
                media=dict(id=ident,kind='replay',role='source',camera=camera,run_id=run['id'],ready=True,mime_type='video/mp4',url=url))
            from .recording_quality import view_for_playback
            result['recording_quality']=view_for_playback(run['directory']/'videos',video_ready=True,
                controller_outcome=value.get('status'))
            rows.append((result,run['directory']/'videos'/filename))
        if not rows:rows=[(dict(id=run['id'],kind='recording',label='推理结果 · 未生成视频',run_id=run['id'],status='invalid',details=dict(recording_error=error,receipt=str(run['directory']/'receipt.json'))),None)]
        with self.lock:
            run['receipt']=receipt;run['recording_error']=error
            self.history.append(rows)

    def allowed(self, action, *, current=None):
        if action in ('refresh_models','inference_refresh_models'):return dict(allowed=True,reason=None)
        if action=='select_result':return dict(allowed=True,reason=None)
        # The page asks every business about every action. Copying the full Run
        # for unrelated actions starves observation publication on this lock.
        if action not in ('select_model','inference_select_model','cancel','inference_prepare','inference_preview','execute'):
            return dict(allowed=False,reason='先准备模型并核对本轮审阅，或等待当前任务结束')
        if current is None:current=self.snapshot()
        if action in ('select_model','inference_select_model'):
            return dict(allowed=not current['busy'],reason='当前推理或数据处理尚未结束' if current['busy'] else None)
        if action=='cancel':return dict(allowed=current['busy'],reason=None if current['busy'] else '当前无进行中的推理')
        allow=not current['busy'] and current['inference']['model_ready'] and (action=='inference_prepare' or action=='inference_preview' and self.prepared is not None and self.cache is not None or action=='execute' and self.status=='awaiting_review')
        return dict(allowed=bool(allow),reason=None if allow else self.cache_error if action=='inference_preview' and self.prepared else '先准备模型并核对本轮审阅，或等待当前任务结束')

    def action(self, action, body):
        if action in ('refresh_models','inference_refresh_models'):return self.refresh_models()
        if action in ('select_model','inference_select_model'):
            return self.select_model(body['model_id'],body.get('expected_bundle_digest'))
        if action in ('inference_prepare','inference_preview'):
            seed=body.get('seed',0)
            if type(seed) is not int or not 0<=seed<2**32:raise ValueError('本轮需要有效 uint32 随机种子')
            duration_s=episode_duration(body.get('duration_s',120))
            if self.device.runtime is None or self.device.runtime.holder!='inference':raise ValueError('请先连接设备并交接给推理业务')
            return self._prepare(seed) if action=='inference_prepare' else self._preview(seed,duration_s,body.get('reference_feedforward_sha256'))
        if action=='execute':
            return self.execute_review(body['token'],physical_clearance=body.get('physical_clearance') is True)
        if action=='cancel':
            if self.device.runtime is not None and self.device.runtime.holder=='inference':
                self.device.submit('inference',{'action':'stop'})
            self.cancel();return dict(message='停止已请求，不会自动继续推理')
        if action=='select_result':return dict(message='结果详情与录像可在当前结果区查看')
        raise ValueError('推理工作区不支持此操作')

    def request_normal_mode(self,body):
        return self._submit_normal_request('normal_mode',body,{'mode','target'})

    def request_operator_stop(self,request_id):
        """One operator intent; the existing Run worker joins HOLD and END."""
        with self.lock:
            run=self.current
            if run is None or run['ended_wall']is not None:
                return dict(accepted=True,no_op=True,reason='no active inference')
            run['operator_stop_request_id']=request_id
            mode=(self.snapshot()['inference'].get('normal_mode')or{})
            if not (mode.get('mode')=='MODEL'and not mode.get('transition_stage')or mode.get('transition_stage')=='RESUMING'):
                return dict(accepted=True,waiting_for_hold=True)
            command=run.get('operator_stop_command')
            if command is None:
                command=dict(session_id=mode['mode_session_id'],runtime_nonce=mode['runtime_nonce'],
                    request_id=request_id,epoch=mode['command_epoch'],mode='HOLD',target='HOLD')
                run['operator_stop_command']=command
        return self.request_normal_mode(command)

    def request_normal_finish(self,body):
        return self._submit_normal_request('normal_finish',body,{'ending_request_id','policy_digest'})

    def _submit_normal_request(self,action,body,fields):
        """Forward once; the Owner owns identity, policy, retries and completion."""
        fields=fields|{'session_id','runtime_nonce','request_id','epoch'}
        if not isinstance(body,dict) or set(body)!=fields:raise ValueError('exact explicit normal mode/finish identity and fields required')
        with self.lock:
            if self.normal_modes is None or self.current is None or not self.current['config'].get('normal_modes'):
                raise ValueError('normal modes are not enabled for this reviewed run')
            if action=='normal_finish'and(self.current['config'].get('normal_finish_kind')!='maintenance_home_v1'
                    or not self.current['config'].get('normal_finish_policy')):
                raise ValueError('maintenance finish is not configured for this reviewed run')
        runtime=self.device.runtime
        if runtime is None or runtime.holder!='inference':raise ValueError('inference does not own the device')
        return self.device.submit('inference',dict(body,action=action))

    def invalidate(self):
        with self.lock:
            self.generation+=1
            if self.status=='awaiting_review':self.status='ready';self.stage='inference.idle';self.message='旧推理审阅已撤销，请重新准备本轮。'

    def cancel(self):
        self.invalidate()

    def snapshot(self):
        with self.lock:
            self._sync_control_policy()
            evidence=self.input_evidence.snapshot()if self.input_evidence is not None else dict(pending=0,error=None,source='resident_ros_recorder')
            run=self.current;value=run['runner'].snapshot() if run and run['runner'] else {}
            active=value.get('runner_status') in ('ARMED','PREPARING','RUNNING','VERIFYING','SETTLING') or bool(value.get('transition_stage'))
            saving=bool(value.get('recording_pending'))
            status='running' if active or saving else self.status
            stage='inference.execute' if active else 'inference.saving' if saving else self.stage
            message=self.message
            if active and (value.get('mode')=='FINISH'or str(value.get('transition_stage','')).startswith('FINISH')):
                stage='inference.finish';message='正在准备或执行收尾回零，完成后保持设备连接。'
            if run and value and not active and not saving:
                status='completed' if value.get('runner_status')=='READY' else 'failed' if value.get('runner_status')=='FAULT' else 'stopped'
                stage='inference.complete';message=value.get('reason') or '本轮推理已结束'
            metrics=deepcopy(run['metrics']) if run else {}
            metrics.update({key:value.get(key) for key in ('elapsed_source_s','remaining_source_s')})
            metrics.update(submitted=value.get('submitted_commands',0),completed=value.get('completed_commands',0),
                elapsed_wall_s=(run['ended_wall'] or time.monotonic())-run['started_wall'] if run and run['started_wall'] is not None else None)
            if active and metrics.get('consecutive_discarded_predictions',0):
                reason=metrics.get('last_discard_reason')
                if reason=='stale_observation':
                    age=metrics.get('last_observation_age_source_s');wall=metrics.get('last_model_wall_s')
                    maximum=run['config']['maximum_observation_age_s']
                    message=f'模型已返回，但观测已超出本轮 {maximum:g} 秒时效，预测已丢弃，正在等待可用的新预测；尚未下发这批动作。'
                    if age is not None:message+=f' 最近观测年龄 {age:.3f} 秒。'
                    if wall is not None:message+=f' 模型请求总耗时 {wall:.3f} 秒。'
                elif reason=='revoked_generation':message='模型已返回，但本轮执行已撤销，迟到预测已丢弃，未下发这批动作。'
                else:message='模型已返回但预测已丢弃（'+str(reason)+'），未下发这批动作。'
            outcome=value.get('outcome') or dict(success=None,reason='尚无末段稳定托盘支撑结论')
            recording_error=(run['recording_error'] or value.get('recording_error')) if run else None
            sealed=(run['receipt'].get('recording') or {}).get('manifest',{}) if run and run['receipt'] else {}
            data='invalid' if recording_error or sealed and sealed.get('capture_status')!='complete' else 'recording' if saving or active else 'valid' if sealed.get('capture_status')=='complete' else 'not_started'
            if self.recording_backend:
                recorder=run.get('recorder') if run else None
                saving=bool(recorder and not recorder.done.is_set())
                data='recording' if saving else (run['receipt']['artifact_status'] if run and run.get('receipt') else 'not_started')
            elif not self.recording_enabled:data='not_applicable';message+=' 本轮录制已关闭。'
            if evidence['error']:message+=' 原始推理证据写入失败：'+evidence['error']
            primary='inference_prepare' if self.prepared is None else 'execute' if self.status=='awaiting_review' else 'inference_preview'
            ingress_closed=bool(run and getattr(run.get('recorder'),'ingress_closed',False))
            busy=self.busy or active or saving and not ingress_closed
            if saving and ingress_closed and not active:message='控制已保持，旧录像正在后台校验与导出；可准备下一轮。'
            operator_task=deepcopy(getattr(self,'operator_task',{}))
            if operator_task.get('status')=='running'and operator_task.get('phase')!='waiting_inference_hold':busy=True
            model_gate=self._model_gate();model_ready=model_gate['ready'] is True
            primary_reason=('原始推理证据写入失败，请先检查存储'if evidence['error'] and not self.recording_backend else
                '当前推理或数据处理尚未结束' if busy else
                model_gate['reason'] if not model_ready else
                self.cache_error or '等待新鲜三相机观测' if primary=='inference_preview' and self.cache is None else None)
            control_ready=self._control_ready()
            duration_s=run['config'].get('execution_duration_s',run['config']['duration_s']) if run else self.duration_s
            return dict(id=run['id'] if run else 'inference-unstarted',status=status,busy=busy,stage_id=stage,title='推理工作区',message=message,
                primary_action=dict(id=primary,label={'inference_prepare':'核对已准备模型与设备','inference_preview':'准备本轮推理审阅','execute':f'确认执行本轮 {duration_s:g} 秒推理'}[primary],allowed=primary_reason is None,reason=primary_reason),
                outcomes=dict(motion='running' if active else 'completed' if value.get('runner_status')=='READY' else status if run else 'not_started',
                    data=data,simulation='not_applicable',task='passed' if outcome.get('success') is True else 'failed' if outcome.get('success') is False else 'unknown'),
                inference=dict(models=deepcopy(self.models),selected_model_id=self.selected_model_id,catalog_root=self.catalog_root,
                    wam_stall_recovery=dict(enabled=bool(run and run.get('stall_recovery')),available=self.wam_stall_recovery_available,
                        **(run['stall_recovery'].snapshot()if run and run.get('stall_recovery')else{})),
                    operator_task=operator_task,operator_controls=deepcopy(getattr(self,'operator_controls',{})),
                    operator_input=deepcopy(getattr(self,'operator_input_status',{})),
                    backend=self.selected_spec.get('backend','torch'),
                    inference_options=deepcopy(self.selected_spec.get('inference_options')),
                    inference_capabilities=deepcopy(self.selected_spec.get('inference_capabilities')),
                    catalog_error=self.catalog_error,bundle_digest=self.selected_bundle_digest,generation=self.generation,
                    model_ready=model_ready,control_ready=control_ready,recording_enabled=bool(self.recording_enabled or self.recording_backend),
                    recording_backend=self.recording_backend,
                    recording_ingress_closed=ingress_closed,
                    background_recordings=[dict(run_id=identity,ingress_closed=getattr(capture,'ingress_closed',False),
                        state=capture.snapshot().get('state'),error=capture.error)
                        for identity,capture in self.pending_captures.items()if capture._finishing and not capture.done.is_set()],
                    execute_steps=run['config']['execute_steps'] if run and 'execute_steps' in run['config'] else self.execute_steps,
                    model=dict(id=self.selected_model_id,label=self.selected_spec['label'],loaded=model_ready,model_ready=model_ready,
                    backend=self.selected_spec.get('backend','torch'),
                    bundle_digest=self.selected_bundle_digest,message=self.message,
                    status='loading' if self.status=='loading' else 'ready' if model_ready else 'error' if self.status=='failed' else
                        'unknown' if model_gate.get('ready') is None else self.model_service_fact.get('status','not_loaded') if self.model_identity else 'not_loaded',
                    readiness=model_gate,error=None if model_ready else model_gate['reason'] or self.model_error),
                    task=self.task,duration_s=duration_s,source_duration_s=run['config']['duration_s'] if run else self.duration_s,
                    prepared_reference_execution=self.prepared_reference_execution,
                    reference_planning=deepcopy(self.reference_planning),
                    finite_chunk_count=self.finite_chunk_count,
                    finite_chunks=deepcopy(value.get('finite_chunks')) if run else None,
                    normal_modes_enabled=self.normal_modes is not None,
                    normal_mode={key:deepcopy(value.get(key)) for key in ('mode','mode_session_id','mode_receipts',
                        'transition_stage','finish_capability','command_epoch','control_state','feedback_healthy',
                        'runner_status','runtime_nonce','reason','normal_model_segment')}
                        if run and self.normal_modes is not None else None,
                    seed=run['config']['seed'] if run else self.seed,metrics=metrics,outcome=deepcopy(outcome),
                    audit=deepcopy(run['audit']) if run else {},observation_error=self.cache_error,recording_error=recording_error,
                    observation_pairing=deepcopy((self.cache or {}).get('pairing')),input_evidence=evidence))

    def results(self):
        return self.results_page()['results']

    def results_page(self, *, before=None, after=None):
        return self.history.page(before=before, after=after)

    def recording_files(self,run_ids=None):
        with self.lock:
            run=self.current;active=[]
            if run and(run.get('ended_wall')is None or run.get('recorder')and not run['recorder'].done.is_set()):
                active.append(run['id'])
        return self.history.recordings(active=active)if run_ids is None else self.history.delete_recordings(run_ids,active=active)

    def media(self, identity):
        if getattr(self,'capture_results',None) is not None and identity.startswith('capture-'):
            return self.capture_results.media('inference',identity)
        return self.history.media(identity)

    def close(self):
        if getattr(self,'operator_input_closed',None)is not None:
            self.operator_input_closed.set();self.operator_input_thread.join(2)
        self.model_probe_stop.set();self.model_probe_thread.join(3)
        self.model_catalog_thread.join(3)
        self.cache_stop.set()
        if self.cache_thread:self.cache_thread.join(2)
        if self.ros_worker is not None:self.ros_worker.close()
        if self.client:self.client.close()
        if self.model_client and self.model_client is not self.client:self.model_client.close()
        if self.input_evidence is not None:self.input_evidence.close()
        run=self.current
        if run and run['config'].get('recording_backend')=='capture_session_v1' and run.get('recorder'):
            run['recorder'].close()
        if self.ros_node is not None:self.ros_node.close()
        if self.artifacts is not None:self.artifacts.close()
        if self.capture_results is not None:self.capture_results.close()
