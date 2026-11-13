#!/usr/bin/env python3
"""One SDK owner; default simulation. Native SDK blocks cannot be interrupted.

A pedal stop retains damping; second press selects manual, never resumes motion.
Only a new, physically reviewed plan can enable position control.
"""
import argparse
from collections import deque
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
import json
import math
import os
from pathlib import Path
import queue
import secrets
import signal
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'infra'))
from arx_env.joint_head_jaw_mapping import command_sdk, feedback_gaps, hold_command_sdk, mapping_identity, FEEDBACK_MAX_MM,command_max_mm,feedback_min_mm,feedback_max_mm
from arx_env.joint_head_real_motion import MotionRunner, make_plan, make_workbench_plans, ARM_SLICES, StationaryWindow, MotionTrackingError, ControlTickGapError, arrival_tolerances, active_indices, feedback_to_command, observe_arm_positions
from robot_workbench.joint_bounds import position_noise_rad, feedback_position_margin_rad, JointPositionBoundaryError
from robot_workbench.execution_settings import JointSpeedProtectionError,NativeCommandMismatchError,FEEDBACK_TIMEOUT_S,NATIVE_WATCHDOG_TIMEOUT_S,DEFAULT_JOINT_TOLERANCE_RAD
from robot_workbench.sdk_process import ReferenceWindowRejected,SDKCommandRejected,SDKNativeFault,validate_native_faults,native_support_confirmed
from robot_workbench.target_interpreter import ReferenceFeedbackStaleError,ReferenceRevokedError


def legacy_modules(source):
    source = Path(source)
    if not source.is_absolute():
        raise ValueError('--legacy-source must be an absolute reviewed source root')
    package = source.resolve(strict=True) / 'infra/arx_env'
    for file in ('coffee_safety.py', 'pedal_stop.py', 'backends/arx5_backend.py'):
        if not (package / file).is_file():
            raise ValueError(f'legacy source missing {file}')
    import arx_env
    arx_env.__path__.append(str(package))
    return importlib.import_module('arx_env.coffee_safety')


def hardware_backend(*, passive_left_jaw=False, disabled_grippers=(), right_j4_diagnostic_kp=None, right_j4_diagnostic_kd=None, left_j6_diagnostic=False, prepared_reference_enabled=False, target_interpreter_profile=None, motion_gain_profile=None, native_reference_api='legacy_recompute', dm_joint_velocity_wire_scale=None, dm_torque_unit_contract=None, ec_torque_unit_contract=None, gravity_calibration=None, can_interfaces=None, gripper_calibration=None,raw_state_enabled=False):
    # Importing this factory does not initialize CAN; only connect() does.
    from arx_env.backends.arx5_backend import Arx5Backend, Arx5BackendConfig
    if type(raw_state_enabled)is not bool:raise ValueError('explicit Raw feedback snapshot flag required')
    import numpy as np
    interfaces = {'left':'can3','right':'can1'} if can_interfaces is None else dict(can_interfaces)
    if interfaces not in ({'left':'can3','right':'can1'},{'left':'can13l','right':'can13r'}):
        raise ValueError('unreviewed physical CAN role mapping')
    if dm_joint_velocity_wire_scale is not None and (type(dm_joint_velocity_wire_scale)not in(int,float)or dm_joint_velocity_wire_scale!=1.5):
        raise ValueError('only the measured45/30 DM joint velocity contract is qualified')
    from robot_workbench.sdk_quiesce import EC_STATIC_UNITS,EC_J2_EQUIVALENT_UNITS,EC_J2_GRAVITY_SCALES,DM_EQUIVALENT_UNITS,DM_LEGACY_TO_PROTOCOL
    if dm_torque_unit_contract is not None and (dm_torque_unit_contract!=DM_EQUIVALENT_UNITS
            or interfaces!={'left':'can13l','right':'can13r'}or dm_joint_velocity_wire_scale!=1.5
            or ec_torque_unit_contract is not None):
        raise ValueError('reviewed ARX13 equivalent DM contract with legacy EC units required')
    if ec_torque_unit_contract not in (None,EC_STATIC_UNITS,*EC_J2_GRAVITY_SCALES):
        raise ValueError('unknown EC torque unit contract')
    if gravity_calibration is not None and ec_torque_unit_contract not in(None,*EC_J2_GRAVITY_SCALES):
        raise ValueError('calibrated gravity permits only the reviewed left J2 unit profiles')
    if ec_torque_unit_contract in EC_J2_GRAVITY_SCALES and ec_torque_unit_contract!=EC_J2_EQUIVALENT_UNITS and gravity_calibration is None:
        raise ValueError('J2 gravity dose requires calibrated gravity')
    if any(side not in ('left','right') for side in disabled_grippers):
        raise ValueError('disabled grippers must name left or right')
    disabled = tuple(side for side in ('left','right')
                     if side in disabled_grippers or side == 'left' and passive_left_jaw)
    if right_j4_diagnostic_kp is not None and (
            type(right_j4_diagnostic_kp) not in (int,float) or right_j4_diagnostic_kp not in (60.,80.)
            or disabled!=('right',)):
        raise ValueError('diagnostic profile permits only reviewed right J4 kp60/80 with right jaw disabled')
    if right_j4_diagnostic_kd is not None and (
            type(right_j4_diagnostic_kd) not in (int,float) or right_j4_diagnostic_kp is None
            or (right_j4_diagnostic_kp,right_j4_diagnostic_kd) not in ((60.,1.),(80.,1.),(80.,1.6))):
        raise ValueError('diagnostic kd requires the reviewed kp/kd pair: 60/1, 80/1 or 80/1.6')
    if type(left_j6_diagnostic)is not bool or left_j6_diagnostic and (disabled!=('right',)or right_j4_diagnostic_kp is not None):
        raise ValueError('left J6 diagnostic requires only its fixed 40/1 profile and disabled right jaw')
    diagnostic_kp=40. if left_j6_diagnostic else right_j4_diagnostic_kp
    diagnostic_kd=1. if left_j6_diagnostic or right_j4_diagnostic_kd is None else right_j4_diagnostic_kd
    diagnostic_side,diagnostic_joint=('left',5)if left_j6_diagnostic else('right',3)
    if type(prepared_reference_enabled)is not bool or prepared_reference_enabled and diagnostic_kp is not None:
        raise ValueError('prepared reference requires a normal, explicitly enabled hardware profile')
    from robot_workbench.target_interpreter import PROFILES, gains as motion_gains
    if motion_gain_profile not in (None,'normal_X5'):
        raise ValueError('explicit normal_X5 motion gains or inherited profile required')
    selected_gain_profile=motion_gain_profile or target_interpreter_profile or 'normal_X5'
    if target_interpreter_profile is not None and (target_interpreter_profile not in PROFILES
            or not prepared_reference_enabled or diagnostic_kp is not None):
        raise ValueError('shared target interpreter requires a reviewed common motion profile, not one-shot gains')
    from robot_workbench.motion_reference import NATIVE_REFERENCE_APIS
    if native_reference_api not in NATIVE_REFERENCE_APIS or native_reference_api=='preserve_reference_v1'and not target_interpreter_profile:
        raise ValueError('known native reference API and shared profile required')

    class PassiveJawController:
        def __init__(self, controller, release=lambda: True):
            self.controller = controller
            self.release = release
            self.retained_jaw = lambda: None
        def __getattr__(self, name):
            return getattr(self.controller, name)
        def _read_native(self, name, *args):
            context=self.__dict__.get('_read_probe')
            if context is None:return getattr(self.controller,name)(*args)
            probe,side=context;started=time.monotonic();cpu=time.thread_time_ns()
            try:return getattr(self.controller,name)(*args)
            finally:
                if len(probe['calls'])<32:
                    probe['calls'].append(dict(side=side,phase=probe['phase'],method=name,
                        started_s=started,wall_s=time.monotonic()-started,cpu_s=(time.thread_time_ns()-cpu)/1e9))
        def get_joint_state(self):return self._read_native('get_joint_state')
        def get_joint_cmd(self):return self._read_native('get_joint_cmd')
        def get_control_observation(self):return self._read_native('get_control_observation')
        def get_gain(self):return self._read_native('get_gain')
        def get_timestamp(self):return self._read_native('get_timestamp')
        def set_joint_traj(self,frames):return self._read_native('set_joint_traj',frames)
        def set_joint_reference(self,frames):return self._read_native('set_joint_reference',frames)
        def set_gain(self, gain):
            if self.release():gain.gripper_kp = gain.gripper_kd = 0.
            elif self.retained_jaw() is not None:
                retained=self.retained_jaw()
                gain.gripper_kp, gain.gripper_kd=retained['kp'],retained['kd']
            return self.controller.set_gain(gain)
        def set_joint_cmd(self, cmd):
            if self.release():cmd.gripper_vel = cmd.gripper_torque = 0.
            return self.controller.set_joint_cmd(cmd)

    @contextmanager
    def native_read_probe(controllers):
        probe=dict(phase='guard',calls=[]);wrapped=[]
        for side,controller in controllers.items():
            if isinstance(controller,PassiveJawController):
                controller._read_probe=(probe,side);wrapped.append(controller)
        try:yield probe
        finally:
            for controller in wrapped:controller._read_probe=None

    motion_profile = target_interpreter_profile
    reference_api = native_reference_api

    class SingleArmBackend(Arx5Backend):
        passive_jaws = disabled
        manual_jaws = frozenset()
        target_interpreter_profile = motion_profile
        motion_gain_profile = selected_gain_profile
        native_reference_api = reference_api
        torque_unit_contract = dm_torque_unit_contract or ec_torque_unit_contract or 'legacy_sdk_v1'
        gravity_calibration_id = None if gravity_calibration is None else gravity_calibration['id']
        native_gripper_calibration_id = None if gripper_calibration is None else gripper_calibration['identity']

        def configure_motion_gains(self,value,*,preserve_hold=False):
            # Default remains RAM-only; a reviewed terminal-hold change preserves output.
            if type(preserve_hold)is not bool:raise ValueError('explicit hold preservation flag required')
            if value is not None:
                from robot_workbench.execution_settings import study_defaults,validate_study
                validate_study(dict(study_defaults(),gains=value))
                if motion_profile!='x5_control50_damping_v1':raise ValueError('study gains require V10 profile')
            if preserve_hold:
                self._require_connected()
                if value is None or getattr(self,'study_gain_override',None)is None:
                    raise ValueError('hold gain change requires an existing explicit study configuration')
                changed=[side for side in ('left','right')if value[side]!=self.study_gain_override[side]]
                now=time.monotonic();windows=getattr(self,'_prepared_bias_windows',{})
                if any(side not in windows or windows[side][0][-1]<now for side in changed):
                    raise ReferenceWindowRejected('hold gain change requires a current reference window')
                try:
                    for side in changed:
                        gain=self._arx5.Gain(6)
                        gain.kp()[:],gain.kd()[:]=value[side]['kp'],value[side]['kd']
                        gain.gripper_kp,gain.gripper_kd=5.,.2
                        self.command_guard();self._controllers[side].set_gain(gain)
                    self.command_guard()
                except BaseException:
                    self.set_damping_hold(manual=False)
                    self.reference_cancel_sequence=getattr(self,'reference_cancel_sequence',0)+1
                    raise
                self.study_gain_override=deepcopy(value)
                return dict(configured=True,hardware_write=True,preserved_hold=True,changed_sides=changed)
            self.study_gain_override=deepcopy(value)
            return dict(configured=value is not None,hardware_write=False)

        def motion_gains(self, side):
            override=getattr(self,'study_gain_override',None)
            if override is not None:return override[side]['kp'][:],override[side]['kd'][:]
            return motion_gains(selected_gain_profile, side)

        def _import_arx5(self):
            sdk=super()._import_arx5()
            from robot_workbench.arx_output_guard import require_guard
            contract=getattr(require_guard(),'sdk_contract',None)
            fault_hold = getattr(require_guard(), 'fault_hold', None)
            if fault_hold is not None:
                from robot_workbench.native_fault_hold import SCHEMA
                if getattr(sdk, 'fault_hold_schema', None) != SCHEMA:
                    raise ValueError('native fault-hold SDK capability differs')
            if contract:
                from robot_workbench.sdk_quiesce import verify_module
                verify_module(sdk,contract)
            if reference_api=='preserve_reference_v1'and(not contract
                    or not callable(getattr(sdk.Arx5JointController,'set_joint_reference',None))
                    or not getattr(sdk,'background_home_supported',False)
                    or not all(callable(getattr(sdk.Arx5JointController,name,None))for name in
                               ('start_reset_to_home','reset_to_home_status'))
                    or not isinstance(getattr(sdk,'ReferenceAdmissionRejected',None),type)):
                raise ValueError('pinned SDK with preserved-reference capability required')
            if dm_joint_velocity_wire_scale is not None and not contract:
                raise ValueError('DM velocity codec requires a pinned native SDK')
            if ec_torque_unit_contract is not None and not contract:
                raise ValueError('EC torque contract requires a pinned native SDK')
            if dm_torque_unit_contract is not None and not contract:
                raise ValueError('DM torque contract requires a pinned native SDK')
            if gravity_calibration is not None and not contract:
                raise ValueError('gravity calibration requires a pinned native SDK')
            if gripper_calibration is not None and not contract:
                raise ValueError('gripper endpoint calibration requires a pinned native SDK')
            if not disabled and dm_joint_velocity_wire_scale is None and ec_torque_unit_contract is None and gravity_calibration is None and gripper_calibration is None:return sdk
            left_interface=self.config.left_can
            interfaces={getattr(self.config,side+'_can') for side in disabled}
            class ConfiguredControllers:
                def __getattr__(self,name):return getattr(sdk,name)
                def Arx5JointController(self,robot,controller,interface):
                    if gripper_calibration is not None:
                        side=next((s for s in ('left','right')if interface==getattr(self_backend.config,s+'_can')),None)
                        if (side is None or robot.robot_model!='X5' or not hasattr(robot,'gripper_closed_readout')
                                or self_backend.config.gripper_width_m!=gripper_calibration['sdk_width_m']):
                            raise ValueError('SDK gripper endpoint capability differs from deployment')
                        row=gripper_calibration['sides'][side]
                        robot.gripper_closed_readout=row['closed_readout']
                        robot.gripper_open_readout=row['open_readout']
                        robot.gripper_width=gripper_calibration['sdk_width_m']
                    if gravity_calibration is not None:
                        side=next((s for s in ('left','right')if interface==getattr(self_backend.config,s+'_can')),None)
                        if side is None:
                            raise ValueError('SDK gravity model differs from the calibrated source')
                        from robot_workbench.gravity_calibration import validate_sdk_model
                        validate_sdk_model(robot,gravity_calibration)
                        robot.urdf_path=gravity_calibration['models'][side]['path']
                    if dm_joint_velocity_wire_scale is not None:
                        if (robot.robot_model!='X5'or list(robot.motor_id)!=[1,2,4,5,6,7]
                                or list(robot.motor_type)!=[sdk.MotorType.EC_A4310]*3+[sdk.MotorType.DM_J4310]*3
                                or not hasattr(robot,'dm_joint_velocity_wire_scale')):
                            raise ValueError('SDK/robot does not match measured DM joint velocity codec')
                        robot.dm_joint_velocity_wire_scale=dm_joint_velocity_wire_scale
                    if ec_torque_unit_contract is not None:
                        if (robot.robot_model!='X5' or list(robot.motor_id)!=[1,2,4,5,6,7]
                                or list(robot.motor_type)!=[sdk.MotorType.EC_A4310]*3+[sdk.MotorType.DM_J4310]*3
                                or not hasattr(robot,'ec_torque_protocol_joints')
                                or not hasattr(robot,'ec_protocol_gravity_scale')
                                or list(robot.ec_torque_protocol_joints) or robot.ec_protocol_gravity_scale!=1.):
                            raise ValueError('SDK/robot does not match reviewed EC torque contract')
                        if interface==left_interface:
                            robot.ec_torque_protocol_joints=[1] if ec_torque_unit_contract in EC_J2_GRAVITY_SCALES else[1,2]
                            robot.ec_protocol_gravity_scale=EC_J2_GRAVITY_SCALES.get(ec_torque_unit_contract,1/1.4)
                    if dm_torque_unit_contract is not None:
                        if (not hasattr(robot,'dm_protocol_torque_equivalent')or robot.dm_protocol_torque_equivalent):
                            raise ValueError('SDK lacks the explicit default-off DM torque contract')
                        robot.dm_protocol_torque_equivalent=True
                    if interface in interfaces:
                        # Native NONE skips clear, enable and every gripper CAN
                        # output; zero Python gains alone would still transmit.
                        robot.gripper_motor_type=sdk.MotorType.NONE
                    native = sdk.Arx5JointController(robot,controller,interface)
                    if fault_hold is not None:
                        side = next(s for s in ('left', 'right') if interface == getattr(self_backend.config, s+'_can'))
                        native.bind_fault_hold(fault_hold.fd, ('left', 'right').index(side))
                    return native
            self_backend=self
            return ConfiguredControllers()

        def connect(self):
            from robot_workbench.arx_output_guard import require_guard
            if self.is_connected:return
            guard=require_guard()
            if set(guard.disabled_grippers) != set(disabled):
                raise ValueError('SDK disabled channels differ from the independent CAN guard')
            # Independent CAN protection must acknowledge before the native
            # constructor, which already enables motors and starts output.
            admission=guard.call('begin')
            self.sdk_generation=(admission or {}).get('generation')
            self.output_guard=guard
            try:
                super().connect()
                if getattr(guard,'sdk_contract',None):
                    for side,controller in self._controllers.items():
                        guard.call('register_sdk',side=side,identity=dict(
                            library_build_id=guard.sdk_contract['library_build_id'],
                            controller_instance_id=controller.controller_instance_id()))
                return guard.call('activate')
            except BaseException as error:
                # Existing private trip RPC carries bounded error evidence;
                # the supervisor still terminates before writing any log.
                guard.call('trip',detail=f'{type(error).__name__}: {error}'[:512])
                raise

        def close(self):
            self.release_steps=deque(maxlen=32)
            collecting=False
            def mark(step,status):
                stamp=time.monotonic()
                self.release_steps.append(dict(step=step,status=status,monotonic_s=stamp))
                if collecting:release_trace.mark(step,status,int(stamp*1e9))
            guard=getattr(self,'output_guard',None)
            release_trace=getattr(guard,'release_trace',None)
            if guard is None:
                if self._controllers:raise RuntimeError('unprotected SDK controller cannot be released normally')
                return super().close()
            mark('guard_releasing','started')
            guard.call('releasing')
            if release_trace is not None:
                release_trace.begin();collecting=release_trace.enabled
            mark('guard_releasing','returned')
            try:
                mark('set_to_damping','started')
                self.set_to_damping()
                mark('set_to_damping','returned')
                for side in ('left','right'):
                    mark('pop_'+side,'started')
                    controller=self._controllers.pop(side,None)
                    mark('pop_'+side,'returned')
                    if getattr(guard,'sdk_contract',None):
                        from robot_workbench.sdk_quiesce import quiesce_report
                        mark('quiesce_'+side,'started')
                        report=quiesce_report(controller,self.sdk_generation)
                        mark('quiesce_'+side,'returned')
                        mark('retire_'+side,'started')
                        guard.call('retire_'+side,report=report)
                        mark('retire_'+side,'returned')
                    mark('destroy_'+side,'started')
                    del controller
                    mark('destroy_'+side,'returned')
                    if not getattr(guard,'sdk_contract',None):
                        mark('retire_'+side,'started')
                        guard.call('retire_'+side)
                        mark('retire_'+side,'returned')
                self._robot_configs.clear();self._controller_configs.clear()
                self._control_mode=None;self._closed=True
            except BaseException:
                mark('close','failed')
                guard.call('trip')
                raise
            if self._controllers:raise RuntimeError('native controllers remain after close')
            mark('guard_disarm','started')
            guard.call('disarm');self.output_guard=None
            mark('guard_disarm','returned')

        def set_damping_hold(self, *, manual=False, preserve_jaws=False, **kwargs):
            if type(preserve_jaws)is not bool or preserve_jaws and manual:
                raise ValueError('preserved command jaws require an explicit position hold')
            retained={}
            if preserve_jaws:
                self.command_guard()
                output=self.read_execution_output()
                if any(not 0<=time.monotonic_ns()-row['sample_monotonic_ns']<=FEEDBACK_TIMEOUT_S*1e9
                       for row in output['observations'].values()):
                    raise ReferenceWindowRejected('retained jaw output is stale')
                for side,slot in (('left',6),('right',13)):
                    if side in self.passive_jaws:continue
                    row=output['gains'][side];command=output['q_sdk14'][slot]
                    if (not 0<=command<=self.config.gripper_width_m or row['gripper_kp']<=0
                            or not all(math.isfinite(x)for x in (command,row['gripper_kp'],row['gripper_kd']))):
                        raise ReferenceWindowRejected('retained command jaw state is invalid')
                    retained[side]=dict(command=command,kp=row['gripper_kp'],kd=row['gripper_kd'])
            self._retained_stop_jaws=retained
            try:return self._set_damping_hold(manual=manual,**kwargs)
            finally:self._retained_stop_jaws={}

        def _set_damping_hold(self, *, manual=False, **kwargs):
            if getattr(getattr(self,'_arx5',None),'background_home_supported',False):
                for controller in self._controllers.values():
                    if controller.reset_to_home_status()=='running':controller.set_to_damping()
            # Keep both jaw position gains off throughout manual gravity mode.
            # Wrap before the legacy backend writes gains, avoiding a brief re-clamp.
            self.manual_jaws = set(self.sides) if manual else set()
            for side in ('left', 'right'):
                if not isinstance(self._controllers[side], PassiveJawController):
                    self._controllers[side] = PassiveJawController(
                        self._controllers[side], lambda side=side: side in self.manual_jaws or side in self.passive_jaws)
                self._controllers[side].retained_jaw=lambda side=side:getattr(self,'_retained_stop_jaws',{}).get(side)
            sides=self.sides
            for attempt in range(2):
                faults=self.native_faults()
                try:
                    for side in faults:self._controllers[side].set_to_damping()
                    # Reuse the legacy measured-pose Stop for only the healthy arm.
                    # This worker dispatches serially; restore membership even on failure.
                    self.sides=tuple(side for side in sides if side not in faults)
                    result = super().set_damping_hold(manual=manual, **kwargs)
                    if not manual:
                        for side in self.sides:
                            gain = self._arx5.Gain(6)
                            gain.kp()[:] = np.asarray(self._controller_configs[side].default_kp)*.3
                            gain.kd()[:] = np.minimum(np.asarray(self._controller_configs[side].default_kd)*self.config.teach_kd_scale+1.,4.5)
                            gain.gripper_kp, gain.gripper_kd = .12, .15
                            self._controllers[side].set_gain(gain)
                    break
                except Exception:
                    # A native latch can race the initial report. Retry only the
                    # bounded stop, never a motion, and only for a newly latched arm.
                    if attempt or not set(self.native_faults())-set(faults):raise
                finally:self.sides=sides
            self._prepared_reference=None
            self._prepared_bias_windows={}
            self._joint_diagnostic_gain_active=False
            return result

        def start_native_home(self,side):
            """Native SDK owns interpolation; the worker keeps servicing feedback."""
            self._require_connected()
            self.command_guard()
            if not getattr(self._arx5,'background_home_supported',False):
                raise ReferenceWindowRejected('native Home requires the qualified background-safe SDK build')
            if side not in self.sides:raise ReferenceWindowRejected('native Home requires a connected arm')
            if any(controller.reset_to_home_status()=='running'for controller in self._controllers.values()):
                raise ReferenceWindowRejected('native Home already active')
            self._controllers[side].start_reset_to_home()
            self._prepared_reference=None
            self._prepared_bias_windows={}
            return dict(accepted=True,side=side,started_source_s=time.monotonic())

        def _stop_gripper_command(self,side,width):
            retained=getattr(self,'_retained_stop_jaws',{}).get(side)
            if retained is not None:return retained['command']
            if gripper_calibration is None:return width
            # Validate the measured envelope separately from the command envelope.
            # Raw SDK feedback is never overwritten, including at the calibrated stop.
            raw=[0.]*14;raw[6 if side=='left'else 13]=width
            gap=feedback_gaps(raw)[0 if side=='left'else 1]
            command=hold_command_sdk(side,gap)
            if not hasattr(self,'_stop_jaw_projection'):self._stop_jaw_projection={}
            self._stop_jaw_projection[side]=dict(raw_sdk_m=width,command_sdk_m=command,
                measured_gap_mm=gap,command_clipped=abs(command-width)>1e-12)
            return command

        def _stop_arm_command(self,side,position):
            from robot_workbench.joint_bounds import inspect_joint_feedback
            command,corrections=inspect_joint_feedback(np.asarray(position).tolist(),
                np.asarray(self.get_joint_position_limits(side))[:6].tolist(),environment='physical')
            if not hasattr(self,'_stop_arm_projection'):self._stop_arm_projection={}
            self._stop_arm_projection[side]=dict(raw_sdk_rad=np.asarray(position).tolist(),
                command_sdk_rad=list(command),corrections=corrections)
            return command

        def native_faults(self):
            result={}
            for side,controller in self._controllers.items():
                read=getattr(controller,'get_emergency_state',None)
                if read is None:continue # Existing pinned SDK remains compatible.
                report=read()
                if not report.latched:continue
                contract=getattr(getattr(self,'output_guard',None),'sdk_contract',None)
                if (not contract or report.library_build_id!=contract['library_build_id']
                        or report.controller_instance_id!=controller.controller_instance_id()):
                    raise ValueError('native fault SDK identity differs')
                result[side]={k:getattr(report,k)for k in ('library_build_id','controller_instance_id','reason',
                    'latched','feedback_valid','damping_committed','joint','observed_value','latched_monotonic_ns')}
                if hasattr(report, 'support_mode'):
                    result[side]['support_mode'] = report.support_mode
            return validate_native_faults(result)

        def set_control_mode(self, mode):
            for side in disabled:
                if not isinstance(self._controllers[side], PassiveJawController):
                    self._controllers[side] = PassiveJawController(self._controllers[side])
            return super().set_control_mode(mode)

        def read_execution_output(self):
            """Fresh completed native outputs, including jaws; not motor ACKs."""
            self._require_connected()
            result=dict(q_sdk14=[],dq_sdk14=[],gains={},observations={},firmware_ack=False,
                scope='coherent_native_completed_output_per_arm; not_hardware_atomic')
            for side in ('left','right'):
                controller=self._controllers[side]
                read=getattr(controller,'get_control_observation',None)
                instance=getattr(controller,'controller_instance_id',None)
                if not callable(read)or not callable(instance):raise ReferenceWindowRejected('coherent native output capability required; no feedback fallback')
                instance_id=instance()
                if not isinstance(instance_id,str)or not instance_id:raise ReferenceWindowRejected('native controller instance identity required')
                observation=read();stamp=int(observation.sample_monotonic_ns)
                if (not observation.valid or observation.emergency or int(observation.sequence)<=0
                        or not 0<=time.monotonic_ns()-stamp<=100_000_000):
                    raise ReferenceWindowRejected('fresh healthy native output identity required')
                cmd,gain=observation.command,observation.gain
                q=[*map(float,cmd.pos()),float(cmd.gripper_pos)]
                dq=[*map(float,cmd.vel()),float(cmd.gripper_vel)]
                row=dict(kp=list(map(float,gain.kp())),kd=list(map(float,gain.kd())),
                    gripper_kp=float(gain.gripper_kp),gripper_kd=float(gain.gripper_kd))
                if (len(q)!=7 or len(dq)!=7 or len(row['kp'])!=6 or len(row['kd'])!=6
                        or not all(math.isfinite(x)for x in q+dq+row['kp']+row['kd']+[row['gripper_kp'],row['gripper_kd']])):
                    raise ReferenceWindowRejected('finite complete native command and gains required')
                result['q_sdk14'].extend(q);result['dq_sdk14'].extend(dq);result['gains'][side]=row
                command_stamp=getattr(cmd,'timestamp',None)
                result['observations'][side]=dict(controller_instance_id=instance_id,sequence=int(observation.sequence),sample_monotonic_ns=stamp,
                    command_sdk_time_s=(float(command_stamp)if isinstance(command_stamp,(int,float))and math.isfinite(command_stamp)else None))
            configs=self._robot_configs
            result['velocity_limits_sdk14']=(
                [float(v)for side in('left','right')for v in(*configs[side].joint_vel_max,configs[side].gripper_vel_max)]
                if all(hasattr(configs[s],'joint_vel_max')and hasattr(configs[s],'gripper_vel_max')for s in('left','right'))else None)
            return result

        def set_execution_gains(self, rows):
            """Bounded normal/teach transition, including passive manual jaws."""
            output=self.read_execution_output()
            if not isinstance(rows,dict)or set(rows)!={'left','right'}:raise ReferenceWindowRejected('both gain rows required')
            for side,row in rows.items():
                if not isinstance(row,dict)or set(row)!={'kp','kd','gripper_kp','gripper_kd'}:
                    raise ReferenceWindowRejected('exact execution gain fields required')
                kp,kd=np.asarray(row['kp']),np.asarray(row['kd'])
                if kp.dtype.kind not in 'fiu'or kd.dtype.kind not in 'fiu':
                    raise ReferenceWindowRejected('numeric execution gain vectors required')
                kp,kd=kp.astype(float),kd.astype(float)
                cap_kp,cap_kd=motion_gains('normal_X5',side)
                config=self._controller_configs[side]
                if (kp.shape!=(6,)or kd.shape!=(6,)or not np.isfinite(kp).all()or not np.isfinite(kd).all()
                        or np.any(kp<0)or np.any(kd<0)or np.any(kp>cap_kp)or np.any(kd>cap_kd)
                        or any(type(row[k])not in(int,float)or not math.isfinite(row[k])or not 0<=row[k]<=float(np.float32(getattr(config,'default_'+k)))
                               for k in('gripper_kp','gripper_kd'))):
                    raise ReferenceWindowRejected('bounded normal joint and jaw gains required')
            try:
                for side,row in rows.items():
                    gain=self._arx5.Gain(6);gain.kp()[:]=row['kp'];gain.kd()[:]=row['kd']
                    gain.gripper_kp,gain.gripper_kd=row['gripper_kp'],row['gripper_kd']
                    self.command_guard();self._controllers[side].set_gain(gain)
                self.command_guard()
            except BaseException:
                self.set_damping_hold(manual=False)
                self.reference_cancel_sequence=getattr(self,'reference_cancel_sequence',0)+1
                raise
            return dict(returned=True,firmware_ack=False,gains=deepcopy(rows))

        def begin(self, side, physical14, *, preserve_jaws=False):
            self._require_connected()
            if type(preserve_jaws)is not bool:raise ValueError('explicit begin jaw policy required')
            output=None
            if preserve_jaws:
                if getattr(self,'_prepared_reference',None)is not None:
                    raise ReferenceWindowRejected('strict begin cannot replace an existing reference; use its continuation')
                output=self.read_execution_output()
            if diagnostic_kp is not None and (side!=diagnostic_side or getattr(self,'_joint_diagnostic_consumed',False)):
                raise ValueError('diagnostic profile permits exactly one '+diagnostic_side+'-arm begin; no replay')
            self.manual_jaws = set(self.manual_jaws) - {side}
            start, end = ARM_SLICES[side]
            cfg = self._controller_configs[side]
            if (self._robot_configs[side].robot_model != 'X5' or not self.config.gravity_compensation
                    or not np.allclose(cfg.default_kp, [80, 70, 70, 30, 30, 20], rtol=0, atol=1e-9)
                    or not np.allclose(cfg.default_kd, [2, 2, 2, 1, 1, .7], rtol=0, atol=1e-9)
                    or not np.isclose(cfg.default_gripper_kp, 5., rtol=0, atol=1e-6)
                    or not np.isclose(cfg.default_gripper_kd, .2, rtol=0, atol=1e-7)):
                raise ValueError('X5 gain identity changed')
            measured = self._controllers[side].get_joint_state()
            q, jaw = list(map(float, measured.pos())), float(measured.gripper_pos)
            if (len(q) != 6 or not all(math.isfinite(x) for x in q + [jaw])
                    or max(abs(a-b) for a, b in zip(q, physical14[start:end])) > DEFAULT_JOINT_TOLERANCE_RAD
                    or side not in disabled and abs(feedback_gaps([jaw if i==end else physical14[i] for i in range(14)])[0 if side=='left' else 1]-feedback_gaps(physical14)[0 if side=='left' else 1]) > 2.):
                raise ValueError('measured begin pose changed')
            exact = list(physical14)
            exact[start:end], exact[end] = q, jaw
            corrections=[]
            hold = list(feedback_to_command(exact, np.asarray(self.get_bimanual_joint_position_limits(),dtype=float).tolist(),
                environment='physical',corrections=corrections))
            if output is not None:
                hold[end]=output['q_sdk14'][end]
                if not 0<=hold[end]<=self.config.gripper_width_m:
                    raise ReferenceWindowRejected('retained native jaw command outside command limits')
            self.last_begin_position_boundary=dict(raw_qpos=list(exact),command_qpos=hold[:],corrections=corrections,
                environment='physical',position_noise_rad=position_noise_rad('physical'))
            if diagnostic_kp is not None:self._joint_diagnostic_consumed=True
            if output is not None and any(not 0<=time.monotonic_ns()-row['sample_monotonic_ns']<=100_000_000
                    for row in output['observations'].values()):
                raise ReferenceWindowRejected('native jaw command evidence expired before begin output')
            self._send_fixed(side, hold)  # Lifecycle hold before position gains, never ordinary point execution.
            gain = self._arx5.Gain(6)
            gain.kp()[:], gain.kd()[:] = self.motion_gains(side)
            if diagnostic_kp is not None:
                gain.kp()[diagnostic_joint]=diagnostic_kp
                gain.kd()[diagnostic_joint]=diagnostic_kd
            gain.gripper_kp, gain.gripper_kd = 5., .2
            self.command_guard()
            self._controllers[side].set_gain(gain)
            self._joint_diagnostic_gain_active=diagnostic_kp is not None
            self._control_mode = f'{side}_calibration_position_other_unchanged'
            if output is not None:
                return dict(measured_qpos14=exact,command_sdk14=hold,source_output=output)
            return tuple(exact)

        def read_calibration_state(self):
            with native_read_probe(self._controllers)as probe:return self._read_calibration_state(probe)

        def _read_calibration_state(self,probe):
            # Guard/diagnostics may wait. Acquire the actual control snapshot LAST;
            # never relabel the timestamp of an older snapshot as fresh.
            result={};read_started=time.monotonic()
            if getattr(getattr(self,'_arx5',None),'background_home_supported',False):
                result['native_home']={side:self._controllers[side].reset_to_home_status()for side in self.sides}
            heartbeat=self.output_guard.call('heartbeat')
            guard_returned=time.monotonic()
            faults=self.native_faults()
            if faults:
                result['native_faults']=faults
                if not all(row['feedback_valid'] for row in faults.values()):
                    if not all(native_support_confirmed(row) for row in faults.values()):
                        self.set_damping_hold(manual=False)
                        faults = self.native_faults()
                        result['native_faults'] = faults
                    details = {side:{key:row[key] for key in ('reason','joint','observed_value','latched_monotonic_ns')}
                               for side,row in faults.items()}
                    if not getattr(self, '_native_fault_reported', False):
                        self.output_guard.call('trip',detail=('native emergency: '+json.dumps(details,separators=(',',':')))[:512])
                        self._native_fault_reported = True
                    if not all(native_support_confirmed(row) for row in faults.values()):
                        raise RuntimeError('native emergency: invalid feedback; retained hold unavailable')
                    heartbeat = self.output_guard.call('heartbeat')
            probe['phase']='jaw_diagnostic'
            result['gripper_feedback_valid']=[side not in disabled for side in ('left','right')]
            result['gripper_feedback_reason']=['operator_disabled_no_feedback' if side in disabled else None for side in ('left','right')]
            guard_fault = (heartbeat or {}).get('native_fault_hold')
            result['can_output_guard']=dict(state='latched' if guard_fault else 'armed',feedback_timeout_ms=100,
                heartbeat_monotonic_s=(heartbeat or {}).get('heartbeat_monotonic_s'),
                disabled_grippers=list(disabled),loss_action='native_position_hold' if getattr(self.output_guard,'fault_hold',None) else 'latched_native_process_exit')
            if guard_fault:
                result['can_output_guard'].update(native_fault_hold=guard_fault, reason=heartbeat.get('reason'))
            if not raw_state_enabled:
                result['gripper_torque']=[float(self._controllers[side].get_joint_state().gripper_torque) for side in self.sides]
            result['gripper_contact_detection']='unvalidated; torque telemetry only'
            result['last_begin_position_boundary']=getattr(self,'last_begin_position_boundary',None)
            result['stop_jaw_projection']=deepcopy(getattr(self,'_stop_jaw_projection',None))
            result['stop_arm_projection']=deepcopy(getattr(self,'_stop_arm_projection',None))
            if diagnostic_kp is not None:
                result['left_j6_diagnostic'if left_j6_diagnostic else'right_j4_diagnostic']=dict(kp=diagnostic_kp,kd=diagnostic_kd,
                    consumed=getattr(self,'_joint_diagnostic_consumed',False),
                    gain_active=getattr(self,'_joint_diagnostic_gain_active',False),normal_default_kp=20. if left_j6_diagnostic else 30.)
            result['prepared_reference']=getattr(self,'_prepared_reference',None)
            if gravity_calibration is not None:
                result['gravity_calibration']=dict(id=gravity_calibration['id'],
                    joint_offsets_rad=gravity_calibration['joint_offsets_rad'],models=gravity_calibration['models'],
                    convention=gravity_calibration['convention'],application='SDK model origin only; raw q and command unchanged')
            # Sample diagnostics at 10 Hz; never tune gains or alter commands here.
            now=time.monotonic()
            if now-getattr(self,'_telemetry_time',-math.inf)>=.1:
                self._telemetry_time=now
                telemetry=dict(sample_wall_ns=time.time_ns(),sample_time_s=now,
                    scope='native output before CAN encoding; not firmware acknowledgement',arms={},command_sdk_timestamps={})
                probe['phase']='native_telemetry'
                try:
                    for side,controller in self._controllers.items():
                        # Only the native completed-output snapshot pairs the q
                        # used for gravity with the resulting command and gain.
                        native=getattr(getattr(controller,'controller',controller),'get_control_observation',None)
                        if callable(native):
                            observation=controller.get_control_observation()
                            if not observation.valid:
                                raise ValueError('native control observation is not ready')
                            state,cmd,gain=observation.feedback,observation.command,observation.gain
                            sequence=int(observation.sequence);stamp_ns=int(observation.sample_monotonic_ns)
                            age_s=(time.monotonic_ns()-stamp_ns)/1e9
                            if sequence<=0 or stamp_ns<=0 or age_s<0:
                                raise ValueError('invalid native control observation identity')
                            coherence=dict(coherent=True,sequence=sequence,sample_monotonic_ns=stamp_ns,
                                age_s=age_s,emergency=bool(observation.emergency),scope='completed calculation; not motor acknowledgement')
                        else:
                            state,cmd,gain=controller.get_joint_state(),controller.get_joint_cmd(),controller.get_gain()
                            coherence=dict(coherent=False,reason='legacy separate native getters')
                        telemetry.setdefault('observations',{})[side]=coherence
                        row=dict(qpos=list(map(float,state.pos())),effort=list(map(float,state.torque())),
                            output_qpos=list(map(float,cmd.pos())),output_qvel=list(map(float,cmd.vel())),output_torque=list(map(float,cmd.torque())),
                            kp=list(map(float,gain.kp())),kd=list(map(float,gain.kd())))
                        if coherence['coherent']:
                            row['qvel']=list(map(float,state.vel()))
                        if any(len(values)!=6 or not all(math.isfinite(x) for x in values) for values in row.values()):
                            raise ValueError('invalid native controller telemetry')
                        telemetry['arms'][side]=row
                        if dm_torque_unit_contract is not None:
                            telemetry.setdefault('dm_units',{})[side]=dict(contract=DM_EQUIVALENT_UNITS,
                                protocol_joint_indices=[3,4,5],gravity_scale=DM_LEGACY_TO_PROTOCOL,
                                output_torque_units=['legacy_SDK']*3+['protocol_nominal_Nm']*3,
                                feedback_T_nominal_Nm=[x*DM_LEGACY_TO_PROTOCOL for x in row['effort'][3:]],
                                effort_compatibility='original SDK feedback retained for unchanged protection; T is not independent current')
                        if ec_torque_unit_contract is not None:
                            indices=([1] if ec_torque_unit_contract in EC_J2_GRAVITY_SCALES else[1,2]) if side=='left' else []
                            telemetry.setdefault('ec_units',{})[side]=dict(
                                protocol_joint_indices=indices,model_gravity_scale=EC_J2_GRAVITY_SCALES.get(ec_torque_unit_contract,1/1.4) if indices else 1.,
                                effort_compatibility='raw_current_A * 1.4^2; original protection unchanged',
                                ec_motor_current_A=[x/(1.4*1.4) for x in row['effort'][:3]],
                                ec_motor_nominal_torque_Nm=[x/1.4 for x in row['effort'][:3]],
                                output_torque_units=['protocol_Nm' if i in indices else 'legacy_SDK' for i in range(6)])
                        stamp=getattr(cmd,'timestamp',None)
                        telemetry['command_sdk_timestamps'][side]=float(stamp)if isinstance(stamp,(int,float))and math.isfinite(stamp)else None
                    telemetry['status']='observed'
                except Exception as error:
                    telemetry.update(status='unavailable',error=str(error)[:256])
                self._controller_telemetry=telemetry
            result['controller_telemetry']=self._controller_telemetry
            snapshot_started=time.monotonic()
            probe['phase']='raw_snapshot'
            if raw_state_enabled:
                from robot_workbench.raw_state import read_snapshot
                result.update(read_snapshot(self._controllers,self.sides))
                result['raw_state']['jaw_calibration_id']=self.native_gripper_calibration_id
                result['gripper_torque']=[result['raw_state']['arms'][side]['effort'][-1]for side in self.sides]
            else:result.update(super().read_calibration_state())
            result['feedback_read_timing']=dict(guard_call_s=guard_returned-read_started,
                diagnostics_s=snapshot_started-guard_returned,raw_snapshot_s=time.monotonic()-snapshot_started,
                guard_ipc=getattr(self.output_guard,'last_call_timing',None),
                native_calls=probe['calls'],
                order='guard_and_diagnostics_then_actual_snapshot')
            return result

        def enable_collection_arm(self, side, target):
            """The existing collection gain transition, entirely inside the SDK worker."""
            if diagnostic_kp is not None:
                raise ValueError('single-motion diagnostic profile does not permit collection/inference')
            self._require_connected()
            cfg=self._controller_configs[side]
            if (self._robot_configs[side].robot_model!='X5' or not self.config.gravity_compensation
                    or not np.allclose(cfg.default_kp,[80,70,70,30,30,20],rtol=0,atol=1e-9)
                    or not np.allclose(cfg.default_kd,[2,2,2,1,1,.7],rtol=0,atol=1e-9)
                    or not np.isclose(cfg.default_gripper_kp,5.,rtol=0,atol=1e-6)
                    or not np.isclose(cfg.default_gripper_kd,.2,rtol=0,atol=1e-7)):
                raise ValueError('X5 gain identity changed')
            self.manual_jaws=set(self.manual_jaws)-{side}
            self._send_fixed(side,target)
            stamp=time.monotonic()
            gain=self._arx5.Gain(6)
            gain.kp()[:],gain.kd()[:]=self.motion_gains(side)
            gain.gripper_kp,gain.gripper_kd=5.,.2
            self.command_guard();self._controllers[side].set_gain(gain)
            self._control_mode='both_collection_position'
            return dict(send_monotonic_s=stamp)

        def send(self, side, physical14):
            if target_interpreter_profile is not None:
                raise ValueError('ordinary motion requires the shared target interpreter; point-send is disabled')
            return self._send_fixed(side,physical14)

        def _send_fixed(self, side, physical14):
            self._require_connected()
            start, end = ARM_SLICES[side]
            q, jaw = np.asarray(physical14[start:end], dtype=float), float(physical14[end])
            limits = self.get_joint_position_limits(side)[:6]
            if (q.shape != (6,) or not np.isfinite(q).all() or np.any(q < limits[:, 0])
                    or np.any(q > limits[:, 1]) or not math.isfinite(jaw)
                    or not 0 <= jaw <= self.config.gripper_width_m + 1e-8):
                raise ValueError('invalid single-arm target')
            cmd = self._arx5.JointState(6)
            cmd.pos()[:], cmd.vel()[:], cmd.torque()[:] = q, 0., 0.
            cmd.gripper_pos, cmd.gripper_vel, cmd.gripper_torque = jaw, 0., 0.
            self.command_guard()
            self._controllers[side].set_joint_cmd(cmd)
            # Native fixed output replaced this arm's future torque window.
            getattr(self,'_prepared_bias_windows',{}).pop(side,None)

        @property
        def execution_context_capabilities(self):
            from robot_workbench.sdk_process import EXECUTION_CONTEXT_SCHEMA
            return ({'schema': EXECUTION_CONTEXT_SCHEMA} if prepared_reference_enabled
                    and target_interpreter_profile and reference_api == 'preserve_reference_v1' else None)

        def bind_execution_context(self, value):
            from robot_workbench.execution_settings import validate_execution_context
            self._require_connected()
            try: context = validate_execution_context(value)
            except ValueError as error: raise ReferenceWindowRejected(str(error)) from None
            if (self.execution_context_capabilities is None or context['controller_profile'] != target_interpreter_profile
                    or context['jaw_calibration_id'] != self.native_gripper_calibration_id):
                raise ReferenceWindowRejected('execution context differs from installed SDK configuration')
            if 'velocity_source' in context and (target_interpreter_profile != 'x5_control50_damping_v1' or 'limits' in context):
                raise ReferenceWindowRejected('position-only context requires its qualified research profile')
            limits = context.get('limits')
            jaw_scales = None
            if limits is not None:
                if gripper_calibration is None:
                    raise ReferenceWindowRejected('calibrated jaw units required for executable limits')
                jaw_scales = {side: gripper_calibration['sides'][side]['width_m'] * 1000. / gripper_calibration['sdk_width_m']
                              for side in ('left', 'right')}
                if any(not math.isfinite(scale) or scale <= 0 for scale in jaw_scales.values()):
                    raise ReferenceWindowRejected('positive calibrated jaw scale required')
                self.command_guard()
                configs = self._robot_configs
                native = ([float(v) for side in ('left', 'right')
                           for v in (*configs[side].joint_vel_max, configs[side].gripper_vel_max)]
                          if all(hasattr(configs[s], 'joint_vel_max') and hasattr(configs[s], 'gripper_vel_max')
                                 for s in ('left', 'right')) else None)
                if (not isinstance(native, list) or len(native) != 14
                        or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in native)):
                    raise ReferenceWindowRejected('native configured velocity limits required for execution binding')
                for side in ('left', 'right'):
                    start, end = ARM_SLICES[side]
                    if (limits['velocity_limit_rad_s'] > min(native[start:end])
                            or limits['jaw_velocity_limit_mm_s'] / jaw_scales[side] > native[end]):
                        raise ReferenceWindowRejected('execution limits exceed native velocity capability')
            self.command_guard()
            self._execution_context, self._execution_jaw_scales = context, jaw_scales
            return dict(bound=True)

        def queue_reference(self, packet):
            started=time.monotonic();cpu=time.thread_time_ns()
            with native_read_probe(getattr(self,'_controllers',{}))as probe:
                probe.update(phase='validation',stages=[])
                self.last_reference_timing=probe
                last,last_cpu=started,cpu
                def mark(phase=None):
                    nonlocal last,last_cpu
                    now,current=time.monotonic(),time.thread_time_ns()
                    probe['stages'].append(dict(phase=probe['phase'],started_s=last,wall_s=now-last,cpu_s=(current-last_cpu)/1e9))
                    last,last_cpu=now,current
                    if phase is not None:probe['phase']=phase
                try:return self._queue_reference(packet,mark)
                finally:
                    mark();probe.update(started_s=started,returned_s=last,wall_s=last-started,cpu_s=(last_cpu-cpu)/1e9)

        def _queue_reference(self, packet, mark):
            """Admit a short, epoch-gated window; native SDK alone executes it."""
            self._require_connected()
            if not prepared_reference_enabled:raise ReferenceWindowRejected('prepared reference output is not enabled')
            required={'schema','reference_id','host_times_s','q_sdk14','dq_sdk14','active_arm'}
            from robot_workbench.target_interpreter import SCHEMA as common_schema
            context = getattr(self, '_execution_context', None)
            if isinstance(packet, dict) and 'schema' not in packet and context is not None:
                compact_fields = required - {'schema'}
                if set(packet) not in (compact_fields, compact_fields | {'torque_bias_sdk14'}):
                    raise ReferenceWindowRejected('exact bound reference window required')
                packet = dict(packet, schema=common_schema, controller_profile=context['controller_profile'])
                if 'velocity_source' in context: packet['velocity_source'] = context['velocity_source']
                packet.setdefault('torque_bias_sdk14', [[0.] * 14 for _ in packet['host_times_s']])
            common=isinstance(packet,dict)and packet.get('schema')==common_schema
            fields=required|{'controller_profile','torque_bias_sdk14'} if common else required
            if not isinstance(packet,dict)or (set(packet)not in(fields,fields|{'velocity_source'}) if common else set(packet)not in(fields,fields|{'torque_bias_sdk6'})):
                raise ReferenceWindowRejected('exact prepared reference packet required')
            position_only='velocity_source' in packet
            if position_only and (packet['velocity_source']!='sdk_position_only'
                    or target_interpreter_profile!='x5_control50_damping_v1' or reference_api!='preserve_reference_v1'):
                raise ReferenceWindowRejected('explicit shared research SDK position-only contract required')
            packet_api='legacy_recompute' if position_only else reference_api
            from robot_workbench.execution_settings import reference_execution_limits
            limits_policy = (context or {}).get('limits')
            effective_limits = limits_policy or reference_execution_limits()
            velocity_limit = effective_limits['velocity_limit_rad_s']
            acceleration_limit = effective_limits['acceleration_limit_rad_s2']
            if limits_policy is not None and (not common or position_only or packet_api != 'preserve_reference_v1'
                                             or packet['active_arm'] != 'both'):
                raise ReferenceWindowRejected('bound execution limits require preserved shared bimanual windows')
            if common:
                if (target_interpreter_profile is None or packet['controller_profile']!=target_interpreter_profile
                        or packet['active_arm']not in ('left','right','both')):
                    raise ReferenceWindowRejected('shared reference arm or trusted controller profile differs')
            elif target_interpreter_profile is not None:
                raise ReferenceWindowRejected('legacy reference packets are disabled by the shared target profile')
            elif packet['schema']!='prepared_motion_reference.v1'or packet['active_arm']!='left':
                raise ReferenceWindowRejected('current prepared-reference qualification is left arm only')
            sides=('left','right')if packet['active_arm']=='both'else(packet['active_arm'],)
            if any(self._controller_configs[side].interpolation_method!='linear'for side in sides):
                raise ReferenceWindowRejected('only the qualified native linear interpolator is permitted')
            if (not isinstance(packet['reference_id'],str)or len(packet['reference_id'])!=64
                    or any(c not in '0123456789abcdef'for c in packet['reference_id'])):
                raise ReferenceWindowRejected('prepared reference identity required')
            times=np.asarray(packet['host_times_s'],dtype=float);q=np.asarray(packet['q_sdk14'],dtype=float);dq=np.asarray(packet['dq_sdk14'],dtype=float)
            now=time.monotonic()
            if (times.ndim!=1 or not 2<=len(times)<=32 or q.shape!=(len(times),14)or dq.shape!=q.shape
                    or not all(np.isfinite(a).all()for a in (times,q,dq))or np.any(np.diff(times)<=0)
                    or times[0]<now+.003 or times[-1]>now+.35 or times[-1]-times[0]>.3):
                first_ms=(float(times[0])-now)*1000 if times.ndim==1 and len(times) else None
                last_ms=(float(times[-1])-now)*1000 if times.ndim==1 and len(times) else None
                raise ReferenceWindowRejected('bounded, ordered future reference window required; '
                    f'rows={times.size}, time_shape={times.shape}, q_shape={q.shape}, dq_shape={dq.shape}, '
                    f'first_lead_ms={first_ms}, last_lead_ms={last_ms}, admission_monotonic_s={now}')
            from robot_workbench.motion_reference import torque_bias_limits,FEEDFORWARD_SLEW_LIMITS
            bias=np.asarray(packet['torque_bias_sdk14']if common else
                np.c_[packet.get('torque_bias_sdk6',np.zeros((len(times),6))),np.zeros((len(times),8))],dtype=float)
            bound=np.asarray([*torque_bias_limits(target_interpreter_profile,'left'),0.,*torque_bias_limits(target_interpreter_profile,'right'),0.])
            slew=np.asarray([*FEEDFORWARD_SLEW_LIMITS,0.,*FEEDFORWARD_SLEW_LIMITS,0.])
            if bias.shape!=(len(times),14)or not np.isfinite(bias).all()or np.any(np.abs(bias)>bound+1e-12):
                raise ReferenceWindowRejected('reference torque bias exceeds qualified bounded shape or SDK units')
            if position_only and np.any(bias != 0):
                raise ReferenceWindowRejected('SDK position-only comparison does not admit added torque bias')
            inactive=[i for side in ('left','right')if side not in sides for i in range(*ARM_SLICES[side])]
            if inactive and (np.any(bias[:,inactive]!=0.)or np.any(dq[:,inactive]!=0.)):
                raise ReferenceWindowRejected('inactive arm reference must not add velocity or torque')
            if np.any(np.abs(np.diff(bias,axis=0))>np.diff(times)[:,None]*slew+1e-8):
                raise ReferenceWindowRejected('reference torque bias exceeds intra-window slew bound')
            previous_windows=getattr(self,'_prepared_bias_windows',{})
            for side in sides:
                start,end=ARM_SLICES[side];previous=previous_windows.get(side)
                current=(np.array([np.interp(now,previous[0],previous[1][:,j])for j in range(6)])if previous else np.zeros(6))
                if np.any(np.abs(bias[0,start:end]-current)>max(0.,times[0]-now)*slew[start:end]+1e-4):
                    raise ReferenceWindowRejected('reference torque bias exceeds admission slew bound')
            for side in sides:
                start,end=ARM_SLICES[side];limits=np.asarray(self.get_joint_position_limits(side))[:6]
                if (np.any(q[:,start:end]<limits[:,0])or np.any(q[:,start:end]>limits[:,1])
                        or np.any(np.abs(dq[:,start:end])>(np.asarray(self._robot_configs[side].joint_vel_max)if limits_policy is not None else velocity_limit))
                        or np.any(q[:,end]<0)or np.any(q[:,end]>self.config.gripper_width_m+1e-8)
                        or np.any(dq[:,[6,13]]!=0)):
                    raise ReferenceWindowRejected('prepared reference exceeds SDK command bounds')
                if common and limits_policy is None and np.any(np.abs(np.diff(dq[:,start:end],axis=0))>acceleration_limit*np.diff(times)[:,None]+1e-7):
                    raise ReferenceWindowRejected('shared reference exceeds acceleration admission bound')
            planning_observations=[]
            if limits_policy is not None:
                output=self.read_execution_output()
                intervals=np.diff(times)
                thresholds=(('joint_velocity',velocity_limit,'rad/s'),
                            ('joint_acceleration',acceleration_limit,'rad/s²'),
                            ('jaw_velocity',limits_policy['jaw_velocity_limit_mm_s'],'mm/s'))
                for side in sides:
                    start,end=ARM_SLICES[side]
                    elapsed=times[0]-output['observations'][side]['sample_monotonic_ns']/1e9
                    if elapsed<=0:raise ReferenceWindowRejected('native output timestamp must precede the first future point')
                    jaw_scale=self._execution_jaw_scales[side]
                    window_values=(max(float(np.max(abs(dq[:,start:end]))),
                                       float(np.max(abs(np.diff(q[:,start:end],axis=0))/intervals[:,None]))),
                                   float(np.max(abs(np.diff(dq[:,start:end],axis=0))/intervals[:,None])),
                                   float(np.max(abs(np.diff(q[:,end]))*jaw_scale/intervals)))
                    boundary_values=(float(np.max(abs(q[0,start:end]-np.asarray(output['q_sdk14'][start:end]))))/elapsed,
                                     float(np.max(abs(dq[0,start:end]-np.asarray(output['dq_sdk14'][start:end]))))/elapsed,
                                     float(abs(q[0,end]-output['q_sdk14'][end])*jaw_scale)/elapsed)
                    for phase,values in (('within_window',window_values),('current_output_boundary',boundary_values)):
                        deviations=[dict(quantity=name,observed=observed,threshold=limit,unit=unit)
                                    for observed,(name,limit,unit)in zip(values,thresholds)if observed>limit+1e-6]
                        if deviations:
                            observation=dict(side=side,phase=phase,deviations=deviations,
                                first_point_source_s=float(times[0]),native_sample_source_s=output['observations'][side]['sample_monotonic_ns']/1e9,
                                action='record_only')
                            if phase=='current_output_boundary':
                                observation.update(native_q_sdk7=output['q_sdk14'][start:end+1],native_dq_sdk7=output['dq_sdk14'][start:end+1],
                                    first_q_sdk7=q[0,start:end+1].tolist(),first_dq_sdk7=dq[0,start:end+1].tolist())
                            planning_observations.append(observation)
            mark('command_guard')
            try:self.command_guard()
            except BaseException:
                self.set_damping_hold(manual=False)
                self.reference_cancel_sequence=getattr(self,'reference_cancel_sequence',0)+1
                raise
            windows={};clocks={}
            for side in sides:
                mark('clock_'+side)
                controller=self._controllers[side];observations=[]
                for _ in range(3):
                    before=time.monotonic();sdk_now=controller.get_timestamp();after=time.monotonic()
                    observations.append((after-before,before,sdk_now,after))
                _,before,sdk_now,after=min(observations)
                if not math.isfinite(sdk_now)or sdk_now<=.11 or after-before>.002:
                    raise ReferenceWindowRejected('SDK clock observation invalid or exceeded2ms; no window queued')
                offset=sdk_now-(before+after)/2;clocks[side]=dict(offset_s=offset,uncertainty_s=(after-before)/2)
                mark('frames_'+side)
                start,end=ARM_SLICES[side];frames=[]
                for stamp,row,velocity,torque in zip(times,q,dq,bias):
                    cmd=self._arx5.JointState(6);cmd.timestamp=float(stamp+offset)
                    cmd.pos()[:]=row[start:end];cmd.vel()[:]=velocity[start:end];cmd.torque()[:]=torque[start:end]
                    # Retain analytical dq above for admission checks; the native
                    # trajectory API derives its own dq solely from position/time.
                    if position_only:cmd.vel()[:]=0.
                    cmd.gripper_pos=float(row[end]);cmd.gripper_vel=cmd.gripper_torque=0.;frames.append(cmd)
                windows[side]=frames
            try:
                for side,frames in windows.items():
                    mark('commit_'+side)
                    if times[0]<time.monotonic()+.003:
                        raise ReferenceWindowRejected('future reference expired during multi-arm admission')
                    self.command_guard()
                    if packet_api=='preserve_reference_v1':
                        try:self._controllers[side].set_joint_reference(frames)
                        except self._arx5.ReferenceAdmissionRejected as error:
                            raise ReferenceWindowRejected('native preserved reference rejected: '+str(error))from None
                    else:self._controllers[side].set_joint_traj(frames)
                self.command_guard()
            except BaseException:
                mark('rejected_stop')
                # Covers epoch revocation racing with admission. The existing
                # measured-zero-velocity stop also clears pending interpolation.
                self.set_damping_hold(manual=False)
                self.reference_cancel_sequence=getattr(self,'reference_cancel_sequence',0)+1
                raise
            mark('receipt')
            first=clocks[sides[0]]
            self._prepared_reference=dict(reference_id=packet['reference_id'],active_arm=packet['active_arm'],
                queued_until_host_s=float(times[-1]),host_to_sdk_offset_s=first['offset_s'],
                clock_uncertainty_s=first['uncertainty_s'],clock_mappings=clocks,frames=len(times),queued_host_s=time.monotonic(),
                controller_profile=target_interpreter_profile or 'normal_X5',firmware_ack=False)
            if limits_policy is not None:self._prepared_reference['planning_limit_observations']=planning_observations
            self._prepared_reference['native_reference_api']=packet_api
            self._prepared_reference['velocity_source']='sdk_position_only' if position_only else 'prepared_dq'
            if 'torque_bias_sdk6'in packet:self._prepared_reference['torque_bias_last_sdk6']=bias[-1,:6].tolist()
            if common:self._prepared_reference['torque_bias_last_sdk14']=bias[-1].tolist()
            self._prepared_bias_windows=dict(previous_windows)
            for side in sides:
                start,end=ARM_SLICES[side]
                self._prepared_bias_windows[side]=(times.copy(),bias[:,start:end].copy())
            self._control_mode=packet['active_arm']+'_shared_target_reference'if common else'left_prepared_reference_right_hold'
            return dict(self._prepared_reference)

    return SingleArmBackend(Arx5BackendConfig(
        control_mode='damping', left_can=interfaces['left'], right_can=interfaces['right'],
        left_gripper_open_readout=-3.37205, right_gripper_open_readout=-3.45827))


class SimulatedBackend:
    def __init__(self, q=None):
        self.q = [0.] * 14 if q is None else list(q)
        self.is_connected = True
        self.events = []
        self.origin = time.monotonic()

    def read_calibration_state(self):
        now = time.monotonic()
        return dict(qpos=self.q[:], qvel=[0.]*14, sample_time_s=now,
                    sample_wall_ns=time.time_ns(), sdk_timestamps=[now-self.origin, now-self.origin+41.])

    def get_bimanual_joint_position_limits(self):
        return [[0., .082] if i in (6, 13) else [-3.14, 3.5] for i in range(14)]

    def read_gains(self):
        return {'simulation': True}

    def set_damping_hold(self, **kwargs):
        self.events.append(('damping', kwargs))

    def begin(self, side, q):
        self.events.append(('begin', side, list(q)))
        return tuple(q)

    def send(self, side, q):
        a, b = ARM_SLICES[side]
        self.q[a:b+1] = q[a:b+1]
        self.events.append(('send', side, list(q)))

    def close(self):
        self.events.append(('release',))
        self.is_connected = False


class Owner:
    """HTTP only queues commands or reads cache. tick() alone touches the SDK."""
    def __init__(self, backend, safety, *, listener=None, hardware=False, stop_scale=1., event=None, control_clock=None,position_environment=None,preserve_begin_jaws=False):
        self.backend, self.listener, self.hardware = backend, listener, hardware
        if type(preserve_begin_jaws)is not bool:raise ValueError('explicit begin jaw preservation policy required')
        self.preserve_begin_jaws=preserve_begin_jaws;self.last_begin_command=None
        self.position_environment=position_environment or ('physical' if hardware else 'fake')
        position_noise_rad(self.position_environment)
        if control_clock is not None and not callable(control_clock):
            raise TypeError("control_clock must be callable")
        self.control_clock = control_clock
        self.review_wall_deadline = None
        self.accepted_review_age_s = None
        self.last_confirm_rejection = None
        self.arm_tolerance = DEFAULT_JOINT_TOLERANCE_RAD
        self.arm_tolerances = None
        self.control = safety.PedalTakeover(backend, stop_damping_scale=stop_scale)
        self.states = safety.TakeoverState
        self.stop_scale = stop_scale
        self.event = event or (lambda *args, **kwargs: None)
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=16)
        self.pending = {}
        self.stop_requested = False
        self.release_requested = False
        self.epoch = 0
        self.runner = None
        self.arrival_events = deque(maxlen=128)
        self.plan_sequence = 0
        self.arrival_nonce = secrets.token_hex(8)
        self.arrival_pending = None
        self.sequence_remaining = []
        self.sequence_deadline = None
        self.sequence_samples = []
        self.capture_sample = None
        self.capture_wait = None
        self.capture_stable_since = None
        self.capture_recovery_deadline = None
        self.last_capture_ack = None
        self.last_capture_window_ns = 0
        self.recoverable_fault = None
        self.position_boundary_fault = None
        self.consumed = set()
        self.cache = {'pid': os.getpid(), 'state': 'INITIALIZING', 'connected': False}
        self.last_sdk = None
        self.sdk_progress = None
        self.local_gains_cache=None;self.local_gains_checked=0.;self.local_gains_key=None;self.local_gains_wall_ns=None
        self.feedback_healthy = False
        self.stationary_window = StationaryWindow()
        self.stationary = False
        self.diagnostic_history = deque(maxlen=200)
        self.last_fault_diagnostic = None
        self.feedback_read_duration_s = None
        self.last_tick_s = None
        self.tick_gap_s = None
        self.evidence_logging_healthy = True
        self.current_feedback = None
        self.feedback_clock = None
        self.feedback_source_stamp = None
        self.review_press = None
        self.active_epoch = None
        self.backend.command_guard = self.command_gate

    def control_time(self):
        """Execution/feedback clock; review TTL and operational timing remain wall time."""
        return time.monotonic() if self.control_clock is None else self.control_clock()

    def review_age(self):
        if self.accepted_review_age_s is not None:
            return self.accepted_review_age_s
        # Only an injected execution clock needs a separate wall-clock review age.
        if self.control_clock is not None and self.review_wall_deadline is not None:
            return 30. + time.monotonic() - self.review_wall_deadline
        return None

    def pedal_snapshot(self):
        if self.listener is None:
            return (None, False, True)
        with self.listener._stop_lock:
            return (self.listener._last_press_ns, self.listener._key_down, self.listener.input_healthy)

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.cache, allow_nan=False))

    def submit(self, command):
        if not isinstance(command, dict):
            raise ValueError('command object required')
        action = command.get('action')
        with self.lock:
            if action == 'stop':
                if set(command) != {'action'}:
                    if (set(command) != {'action','owner_pid','command_epoch'}
                            or type(command['owner_pid']) is not int or command['owner_pid'] != os.getpid()
                            or type(command['command_epoch']) is not int or command['command_epoch'] != self.epoch):
                        raise ValueError('conditional stop owner or epoch changed')
                self.epoch += 1
                self.pending.clear()
                self.stop_requested = True
                return {'queued': 'stop', 'command_epoch': self.epoch}
            if action == 'release' and set(command) == {'action', 'confirmation'}:
                if command['confirmation'] != 'ARMS_SUPPORTED_RELEASE':
                    raise ValueError('explicit supported-arm release required')
                self.epoch += 1
                self.pending.clear()
                self.release_requested = True
                return {'queued': 'release'}
            if action == 'simulate_pedal' and not self.hardware and set(command) == {'action'}:
                self.epoch += 1
                self.pending.clear()
                self.commands.put_nowait(('pedal', self.epoch, None))
                return {'queued': 'simulate_pedal'}
            if action == 'recover':
                if (set(command) != {'action', 'confirmation', 'owner_pid', 'command_epoch'}
                        or command['confirmation'] != 'ACKNOWLEDGE_MOTION_FAULT'
                        or command['owner_pid'] != os.getpid() or command['command_epoch'] != self.epoch):
                    raise ValueError('explicit acknowledgement of current owner and fault required')
                if not self.recovery_ready():
                    raise ValueError('recoverable motion fault and fresh stationary feedback required')
                self.commands.put_nowait(('recover', self.epoch, self.pedal_snapshot()))
                return {'queued': 'recover'}
            if action == 'capture_ack':
                wait = self.capture_wait
                if (set(command) != {'action','owner_pid','command_epoch','sample_id','capture_ready_wall_ns'}
                        or type(command['owner_pid']) is not int or command['owner_pid'] != os.getpid()
                        or type(command['command_epoch']) is not int or command['command_epoch'] != self.epoch
                        or not wait or command['sample_id'] != wait['sample_id'] or wait['ack_queued']
                        or type(command['capture_ready_wall_ns']) is not int
                        or command['capture_ready_wall_ns'] != wait['ready_wall_ns']
                        or self.capture_recovery_deadline is not None and self.control_time() >= self.capture_recovery_deadline
                        or self.control_time() >= wait['deadline'] or self.stop_requested or self.release_requested
                        or self.cache.get('motion',{}).get('status') != 'AWAITING_CAPTURE'
                        or not self.feedback_healthy or not self.stationary
                        or not 0 <= self.control_time()-self.cache['calibration_feedback']['sample_time_s'] <= FEEDBACK_TIMEOUT_S):
                    raise ValueError('capture ACK requires current owner, epoch and pending stable sample')
                self.command_gate()
                self.commands.put_nowait(('capture_ack', self.epoch, dict(sample_id=wait['sample_id'],capture_ready_wall_ns=wait['ready_wall_ns'])))
                wait['ack_queued'] = True
                return {'queued':'capture_ack','sample_id':wait['sample_id'],'capture_ready_wall_ns':wait['ready_wall_ns']}
            if action in ('preview', 'preview_home', 'preview_arm_home', 'preview_jaw_target', 'preview_both_home', 'preview_workbench','preview_operator_home','preview_operator_hold'):
                expected = {'action'} if action == 'preview_both_home' else {'action', 'active_arm'} if action != 'preview' else {'action', 'active_arm', 'q_target'}
                if action == 'preview' and 'q_active' in command:
                    expected = {'action', 'active_arm', 'q_active', 'max_velocity'}
                if action == 'preview_workbench':
                    expected = {'action', 'waypoints'}
                if action == 'preview_operator_home':
                    expected = {'action','max_velocity'}
                if action == 'preview_operator_hold':expected={'action'}
                if action == 'preview_jaw_target':
                    expected = {'action', 'active_arm', 'sdk_gap_mm'}
                if set(command) != expected:
                    raise ValueError('preview fields do not match selected action')
                current = self.cache
                if (not current.get('feedback_healthy') or not current.get('stationary')
                        or current.get('automatic_enabled') or self.stop_requested or self.release_requested
                        or current.get('command_epoch') != self.epoch
                        or current.get('state') not in ('READY', 'STOPPED_DAMPING_HOLD', 'MANUAL_TAKEOVER')):
                    raise ValueError('fresh stationary healthy owner required')
                stamp = current['calibration_feedback']['sample_time_s']
                if not 0 <= self.control_time()-stamp <= FEEDBACK_TIMEOUT_S:
                    raise ValueError('feedback cache expired')
                press = self.pedal_snapshot()
                if press[1] or not press[2]:
                    raise ValueError('pedal must be released and healthy')
                q_start = current['calibration_feedback']['qpos']
                samples = []
                if action == 'preview_workbench':
                    waypoints = command['waypoints']
                    if not isinstance(waypoints,list) or not waypoints:raise ValueError('nonempty waypoints required')
                    seen = set(); motion = []
                    for row in waypoints:
                        if not isinstance(row,dict):raise ValueError('waypoint object required')
                        capture = row.get('capture',False); sample = row.get('sample_id')
                        if type(capture) is not bool:raise ValueError('capture must be boolean')
                        if capture:
                            if not isinstance(sample,str) or not sample.strip() or len(sample)>200 or sample in seen:
                                raise ValueError('capture requires unique nonempty sample_id')
                            seen.add(sample)
                        elif sample is not None:raise ValueError('sample_id requires capture=true')
                        samples.append(sample if capture else None)
                        motion.append({k:v for k,v in row.items() if k not in ('capture','sample_id')})
                    plans = make_workbench_plans(q_start, motion, current['joint_limits'], created_at_s=self.control_time(),environment=self.position_environment)
                elif action in ('preview_operator_home','preview_operator_hold'):
                    # The SDK adapter is the sole command-output authority.
                    # Preserve its current jaw commands, not measured openings.
                    output=self.backend.read_execution_output()
                    if any(not 0<=self.control_time()-row['sample_monotonic_ns']/1e9<=FEEDBACK_TIMEOUT_S
                           for row in output['observations'].values()):
                        raise ValueError('fresh native command output required for home')
                    start=list(output['q_sdk14']);target=start[:]
                    home=action=='preview_operator_home'
                    for a,b in ARM_SLICES.values():target[a:b]=[0.]*6 if home else q_start[a:b]
                    plans=(make_plan('both',start,target,current['joint_limits'],kind='both_arm_home'if home else'workbench',
                        q_start_is_command=True,max_velocity=command.get('max_velocity'),created_at_s=self.control_time(),
                        environment=self.position_environment),)
                elif action == 'preview_both_home':
                    target = list(q_start)
                    for a, b in ARM_SLICES.values():
                        target[a:b] = [0.]*6
                        side='left' if b==6 else 'right';target[b]=command_sdk(side,command_max_mm(side))
                    plans = (make_plan('both', q_start, target, current['joint_limits'], kind='both_home', created_at_s=self.control_time(),environment=self.position_environment),)
                elif action == 'preview_jaw_target' and command['active_arm'] == 'both':
                    if getattr(self.backend, 'passive_jaws', ()):
                        raise ValueError('both jaws must be active')
                    gap = command['sdk_gap_mm']
                    gaps = gap if isinstance(gap, dict) else dict(left=gap, right=gap)
                    if set(gaps) != {'left', 'right'} or any(type(v) not in (int, float) or not math.isfinite(v) for v in gaps.values()):
                        raise ValueError('finite left and right SDK gaps required')
                    target = list(q_start)
                    target[6], target[13] = gaps['left'] / 1000., gaps['right'] / 1000.
                    plans = (make_plan('both', q_start, target, current['joint_limits'], kind='jaw_target', created_at_s=self.control_time(),environment=self.position_environment),)
                elif action != 'preview':
                    if command['active_arm'] not in ARM_SLICES:
                        raise ValueError('active_arm must be left or right')
                    target = list(q_start)
                    a, b = ARM_SLICES[command['active_arm']]
                    if action in ('preview_jaw_target') and command['active_arm'] in getattr(self.backend, 'passive_jaws', ()):
                        raise ValueError('selected jaw feedback invalid / passive; cannot command jaw')
                    if action == 'preview_jaw_target':
                        gap = command['sdk_gap_mm']
                        if type(gap) not in (int, float) or not math.isfinite(gap):
                            raise ValueError('finite SDK gap required')
                        target[b] = gap / 1000.
                        kind = 'jaw_target'
                    else:
                        target[a:b] = [0.]*6
                        kind = 'home' if action == 'preview_home' else 'arm_home'
                        if kind == 'home':
                            side='left' if b==6 else 'right';target[b]=command_sdk(side,command_max_mm(side))
                    plan = make_plan(command['active_arm'], q_start, target, current['joint_limits'],
                                     kind=kind, created_at_s=self.control_time(),environment=self.position_environment)
                    plans = (plan,)
                else:
                    options = {}
                    target = command.get('q_target')
                    if 'q_active' in command:
                        if command['active_arm'] not in ARM_SLICES:
                            raise ValueError('one active arm required')
                        q_active, speed = command['q_active'], command['max_velocity']
                        if (not isinstance(q_active, list) or len(q_active) != 6
                                or any(type(v) not in (int,float) or not math.isfinite(v) for v in q_active)
                                or type(speed) not in (int,float) or not math.isfinite(speed) or not 0 < speed <= .5):
                            raise ValueError('six finite active-arm positions and speed in (0, 0.5] required')
                        target = list(feedback_to_command(q_start,current['joint_limits'],environment=self.position_environment))
                        a,b = ARM_SLICES[command['active_arm']]
                        target[a:b] = q_active
                        options['max_velocity'] = speed
                    plan = make_plan(command['active_arm'], q_start, target, current['joint_limits'],
                                     created_at_s=self.control_time(),environment=self.position_environment,**options)
                    plans = (plan,)
                token = secrets.token_urlsafe(24)
                self.pending.clear()
                self.pending[token] = (plans, self.epoch, press[0], os.getpid(), samples or [None]*len(plans))
                self.review_wall_deadline = time.monotonic() + 30.
                return dict(token=token, plan=asdict(plans[0]), plans=[asdict(p) for p in plans],
                            disabled_grippers=list(getattr(self.backend, 'passive_jaws', ())),
                            capture_samples=samples, capture_timeout_s=10., capture_hold_s=.5,
                            sequence_id=plans[0].sha256, total_motion_duration_s=sum(p.duration_s for p in plans),
                            owner_pid=os.getpid(), command_epoch=self.epoch,
                            expires_in_s=30, clearance='Operator must review complete swept path; no collision certification')
            if action == 'confirm':
                if (set(command) != {'action', 'token', 'confirmation', 'physical_clearance'}
                        or command['confirmation'] != 'START_REVIEWED_SINGLE_ARM'
                        or command['physical_clearance'] is not True or not isinstance(command['token'], str)):
                    raise ValueError('reviewed single-arm confirmation and physical_clearance=true required')
                review = self.pending.pop(command['token'], None)
                if review is None:
                    raise ValueError('unknown or consumed preview')
                plans, epoch, press, pid, _ = review
                plan = plans[0]
                if (self.review_wall_deadline is None or time.monotonic() > self.review_wall_deadline or epoch != self.epoch
                        or self.stop_requested or self.release_requested or pid != os.getpid()
                        or self.pedal_snapshot() != (press, False, True)):
                    raise ValueError('preview expired or owner/pedal changed')
                self.commands.put_nowait(('confirm', epoch, (review, self.review_wall_deadline)))
                return {'queued': 'confirm', 'plan_sha256': plan.sha256, 'sequence_id': plans[0].sha256}
            raise ValueError('unsupported action or fields')

    def recovery_feedback_valid(self, feedback, limits):
        try:
            command=feedback_to_command(feedback['qpos'],limits,environment=self.position_environment)
            return all(lo<=q<=hi for q,(lo,hi)in zip(command,limits))
        except (KeyError,TypeError,ValueError):
            return False

    def recovery_ready(self):
        f = self.cache.get('calibration_feedback') or {}
        if not self.recovery_feedback_valid(f,self.cache.get('joint_limits')):return False
        boundary=getattr(self,'position_boundary_fault',None)
        if boundary and self.recoverable_fault==boundary['reason']:
            limits=self.cache.get('joint_limits');q=f.get('qpos')
            if (not boundary['stop_hold_return_confirmed'] or getattr(self,'fault_manual',False)
                    or not limits or not q or len(q)!=len(limits)
                    or f.get('sample_time_s',-1)<boundary.get('stop_return_control_s',math.inf)):return False
            try:observe_arm_positions(q,limits,environment=self.position_environment)
            except ValueError:return False
            if self.hardware:
                guard=f.get('can_output_guard')or{};stamp=guard.get('heartbeat_monotonic_s')
                if (guard.get('state')!='armed' or type(stamp)not in(int,float)
                        or not 0<=self.control_time()-stamp<=NATIVE_WATCHDOG_TIMEOUT_S):return False
        return bool(self.recoverable_fault and not getattr(self,'native_faults',{}) and self.control.state is self.states.FAULT
                    and self.backend.is_connected and self.feedback_healthy and self.stationary
                    and not self.stop_requested and not self.release_requested
                    and 0 <= self.control_time()-f.get('sample_time_s', -1) <= getattr(self,'recovery_feedback_limit_s',FEEDBACK_TIMEOUT_S)
                    and self.pedal_snapshot()[1:] == (False, True))

    def invalidate(self, *, normal_transition=False):
        interpreter=getattr(getattr(self,'runner',None),'target_interpreter',None)
        if interpreter is not None:interpreter.revoke()
        with self.lock:
            self.arrival_pending = None
            self.epoch += 1
            self.pending.clear()
            self.sequence_remaining = []
            self.sequence_deadline = None
            self.sequence_samples = []
            self.capture_sample = None
            self.capture_wait = None
            self.capture_stable_since = None
            self.capture_recovery_deadline = None

    def stop(self, reason):
        # A press arriving inside a callback gets its original first/second-press
        # meaning exactly once; a generic fault must not toggle it again.
        if self.listener and self.listener.input_healthy:
            presses = self.listener.take_presses()
            if presses:
                for _ in presses:
                    self.pedal()
                return
        was_fault = self.control.state is self.states.FAULT
        previous_reason = self.control.reason
        self.invalidate()
        self.control.auto_aborted = True
        self.control.reason = previous_reason if was_fault else str(reason)
        self.control.state = self.states.FAULT
        self.backend.set_damping_hold(manual=False, experimental_stop_scale=self.stop_scale)
        self.control.state = self.states.FAULT if was_fault else self.states.STOPPED
        self.event('stop', reason=str(reason), epoch=self.epoch,
                   motion_record=self.runner.motion_record() if isinstance(self.runner,MotionRunner) else None)

    def cancel_requested_reference(self, reason):
        if not isinstance(reason,ReferenceRevokedError):return False
        with self.lock:
            if self.control.state is self.states.FAULT or not (self.stop_requested or self.release_requested):return False
            action='explicit release' if self.release_requested else 'operator stop'
            self.stop_requested=False
        self.stop(action)
        if self.runner:self.runner.status,self.runner.reason='STOPPED',action
        self.event('reference_cancelled',reason=str(reason),action=action,epoch=self.epoch)
        return True

    def motion_fault(self, reason):
        if isinstance(reason,SDKNativeFault):
            self.native_fault(reason.faults)
            return
        if self.cancel_requested_reference(reason):return
        diagnostic = self.diagnostic_sample(self.current_feedback)
        diagnostic.update(reasons=['motion_fault'],error=str(reason))
        if isinstance(reason,NativeCommandMismatchError):
            diagnostic['native_command_mismatch']=deepcopy(reason.diagnostic)
        self.recoverable_fault = None
        self.recovery_feedback_limit_s = reason.maximum_age_s if isinstance(reason,ReferenceFeedbackStaleError)else FEEDBACK_TIMEOUT_S
        self.stop(reason)
        # A runner fault needs diagnosis; new previews do not reset faults.
        if not (self.runner and self.runner.reason == 'physical/operator pedal'):
            self.control.state = self.states.FAULT
            if isinstance(reason, (MotionTrackingError,ControlTickGapError,ReferenceFeedbackStaleError,ReferenceWindowRejected,SDKCommandRejected)):
                self.recoverable_fault = str(reason)
            if isinstance(reason,ControlTickGapError):
                diagnostic['control_tick_gap']=dict(observed_gap_s=reason.gap_s,limit_s=.1,
                    stop_hold_return_confirmed=True,recovery='explicit_ack_after_healthy_stationary_feedback_no_resume')
            if isinstance(reason,(ReferenceWindowRejected,SDKCommandRejected)):
                diagnostic['reference_admission']=dict(rejected=True,stop_hold_return_confirmed=True,
                    recovery='explicit_ack_after_healthy_stationary_feedback_no_resume',
                    timing=deepcopy(getattr(reason,'timing',None)))
            if isinstance(reason,ReferenceFeedbackStaleError):
                diagnostic['reference_feedback']=dict(age_s=reason.age_s,maximum_age_s=reason.maximum_age_s,phase=reason.phase)
            interpreter=getattr(self.runner,'target_interpreter',None)
            if interpreter is not None:
                diagnostic['target_translation_timing']=deepcopy(interpreter.last_timing)
            self.last_fault_diagnostic = diagnostic
            self.event('fault_diagnostic',diagnostic=diagnostic,
                       history=list(self.diagnostic_history),history_capacity=200,epoch_after=self.epoch)

    def native_fault(self, faults):
        faults=validate_native_faults(faults,required=True)
        identity=tuple((s,r['controller_instance_id'],r['latched_monotonic_ns'])for s,r in sorted(faults.items()))
        self.native_faults=faults
        self.recoverable_fault=None
        if identity==getattr(self,'native_fault_identity',None):return
        self.native_fault_identity=identity
        self.invalidate()
        self.control.auto_aborted=True
        self.control.state=self.states.FAULT
        self.control.reason=str(SDKNativeFault(faults))
        if self.runner:self.runner.status,self.runner.reason='FAULT',self.control.reason
        self.last_fault_diagnostic=dict(reasons=['native_emergency_latched'],native_faults=faults,
            stop_return_confirmed=False,recovery='explicit_release_required_no_in_place_clear',epoch_after=self.epoch)
        try:self.backend.set_damping_hold(manual=False)
        except Exception as error:
            self.last_fault_diagnostic['stop_error']=str(error)
            raise
        self.last_fault_diagnostic['stop_return_confirmed']=True
        self.event('fault_diagnostic',diagnostic=self.last_fault_diagnostic)

    def pedal(self):
        self.invalidate()
        if self.runner:
            self.runner.status, self.runner.reason = 'STOPPED', 'physical/operator pedal'
        if self.control.state is self.states.FAULT:
            # Toggle physical damping without clearing the fault or enabling motion.
            manual = not getattr(self, 'fault_manual', False)
            self.backend.set_damping_hold(manual=manual)
            self.fault_manual = manual
            self.control.auto_aborted = True
        else:
            self.control.press()
        self.event('pedal', state=self.control.state.value, epoch=self.epoch)

    def command_gate(self):
        press, down, healthy = self.pedal_snapshot()
        with self.lock:
            blocked = (self.stop_requested or self.release_requested or self.epoch != self.active_epoch)
        if blocked:raise ReferenceRevokedError('stop/release/epoch changed before SDK command')
        if not healthy or down or press != self.review_press:
            raise RuntimeError('stop/pedal/epoch changed before SDK command')

    def begin(self, arm, measured):
        self.command_gate()
        exact = measured
        commanded=None;outputs={}
        for side in (('left', 'right') if arm == 'both' else (arm,)):
            self.command_gate()
            if self.preserve_begin_jaws:
                result=self.backend.begin(side,exact,preserve_jaws=True)
                exact=result['measured_qpos14']
                if commanded is None:commanded=list(result['command_sdk14'])
                start,end=ARM_SLICES[side];commanded[start:end+1]=result['command_sdk14'][start:end+1]
                outputs[side]=result['source_output']
            else:exact = self.backend.begin(side, exact)
        self.command_gate()
        self.last_begin_command=(None if commanded is None else dict(command_sdk14=commanded,
            source_outputs=outputs,epoch=self.epoch,runtime_nonce=getattr(self,'runtime_nonce',None),firmware_ack=False))
        self.control.auto_aborted = False
        self.control.state = self.states.READY
        self.control.reason = 'new explicitly reviewed single-arm motion'
        return exact

    def send(self, arm, desired):
        self.command_gate()
        guard=getattr(self,'control_guard',None)
        if guard is not None:
            feedback=getattr(self,'current_feedback',None)or self.cache.get('calibration_feedback')or{}
            values=[desired[i]for i in active_indices('both')]
            decision=guard.evaluate(dict(rule_id='G18',signal_id='point_command',
                observed=dict(minimum=values,maximum=values,active_arm=arm),
                source_time=feedback.get('sample_time_s'),checked_time=self.control_time(),epoch=self.epoch),'submission')
            self.event('guard_decision',**decision)
            if decision['verdict']=='reject':raise ValueError('G18: point command exceeds the bound device capability')
            from arx_env.joint_head_jaw_mapping import command_gap_mm
            jaws=[command_gap_mm(side,desired[slot])for side,slot in (('left',6),('right',13))]
            decision=guard.evaluate(dict(rule_id='G17',signal_id='point_jaw_command',
                observed=dict(minimum=jaws,maximum=jaws,active_arm=arm),source_time=feedback.get('sample_time_s'),
                checked_time=self.control_time(),epoch=self.epoch),'submission')
            self.event('guard_decision',**decision)
            if decision['verdict']=='reject':raise ValueError('G17: point jaw command exceeds the bound SDK capability')
        for side in (('left', 'right') if arm == 'both' else (arm,)):
            self.command_gate()
            self.backend.send(side, desired)

    def read_feedback(self):
        read_started = time.monotonic()
        feedback = self.backend.read_calibration_state()
        if feedback.get('native_faults'):
            self.native_faults=validate_native_faults(feedback['native_faults'],required=True)
            if any(not row['feedback_valid'] for row in self.native_faults.values()):
                self.feedback_healthy = self.stationary = False
                self.native_fault(self.native_faults)
                return None
        self.feedback_read_duration_s = time.monotonic()-read_started
        now = self.control_time()
        q, v, sdk = feedback['qpos'], feedback['qvel'], feedback['sdk_timestamps']
        if any(key in feedback for key in ('clock_domain','source_clock_id','source_time_ns','physics_step')):
            from robot_workbench.sample_clock import assert_progress, clock_spec, sample_stamp
            environment=getattr(self,'environment',None) or 'physical'
            clock=self.feedback_clock or clock_spec(environment,clock_domain=feedback.get('clock_domain'),
                                                    source_clock_id=feedback.get('source_clock_id'))
            point=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
            assert_progress(self.feedback_source_stamp,point,allow_same=True)
            self.feedback_clock,self.feedback_source_stamp=clock,point
        values = [*q, *v, *sdk, feedback['sample_time_s']]
        if (len(q) != 14 or len(v) != 14 or len(sdk) != 2
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values)
                or not 0 <= now-feedback['sample_time_s'] <= FEEDBACK_TIMEOUT_S or min(sdk) < 0):
            raise ValueError('invalid or stale raw feedback')
        if self.last_sdk is None:
            self.sdk_progress = [None, None]
        else:
            for i, (new, old) in enumerate(zip(sdk, self.last_sdk)):
                if new < old:
                    raise ValueError('SDK clock regressed')
                if new > old:
                    self.sdk_progress[i] = now
        self.last_sdk = list(sdk)
        self.feedback_healthy = all(t is not None and now-t <= FEEDBACK_TIMEOUT_S for t in self.sdk_progress)
        if (feedback.get('can_output_guard') or {}).get('state') == 'latched':
            self.feedback_healthy = False
        if not self.feedback_healthy:
            self.stationary_window.reset()
            self.stationary = False
        else:
            self.stationary = self.stationary_window.update(feedback['sample_time_s'], q, v)
        return feedback

    def diagnostic_sample(self, feedback):
        """Bounded in-memory evidence; the existing async logger writes boundary events."""
        now = self.control_time()
        runner = self.runner
        return dict(schema='joint-head-capture-diagnostic-v1',wall_ns=time.time_ns(),monotonic_s=time.monotonic(),control_time_s=now,
                    owner_pid=os.getpid(),command_epoch=self.epoch,sample_id=self.capture_sample,
                    plan_sha256=runner.plan.sha256 if runner else None,
                    target_qpos14=list(runner.plan.q_target) if runner else None,
                    target_error=dict(runner.target_error) if runner and runner.target_error else None,
                    motion_state=runner.status if runner else None,control_state=self.control.state.value,
                    feedback_healthy=self.feedback_healthy,stationary=self.stationary,
                    stationary_check=getattr(self.stationary_window,'last_diagnostic',None),
                    runner_stationary_check=getattr(runner.stationary,'last_diagnostic',None) if runner else None,
                    arm_arrival_tolerances_rad=list(self.motion_error_settings()['arrival_tolerances_rad']),
                    execution_settings=self.motion_error_settings(),
                    feedback=feedback,tick_gap_s=self.tick_gap_s,feedback_read_duration_s=self.feedback_read_duration_s,
                    sdk_progress_age_s=[None if t is None else now-t for t in (self.sdk_progress or [])],
                    capture_ready_wall_ns=self.capture_wait['ready_wall_ns'] if self.capture_wait else None,
                    capture_ready_source_ns=self.capture_wait.get('ready_source_ns') if self.capture_wait else None,
                    capture_source_clock_id=self.capture_wait.get('source_clock_id') if self.capture_wait else None,
                    capture_ack_remaining_s=self.capture_wait['deadline']-now if self.capture_wait else None,
                    capture_recovery_deadline_monotonic_s=self.capture_recovery_deadline,
                    pedal=self.pedal_snapshot())

    def tick(self):
        """Called at 100 Hz on the owner thread; never from a request handler."""
        feedback = None
        self.current_feedback = None
        tick_started = time.monotonic()
        self.tick_gap_s = None if self.last_tick_s is None else tick_started-self.last_tick_s
        self.last_tick_s = tick_started
        guard_reasons = []
        try:
            with self.lock:
                release, stop = self.release_requested, self.stop_requested
                self.stop_requested = False
            if release:
                if self.runner:
                    self.runner.status, self.runner.reason = 'STOPPED', 'explicit release'
                self.control.release()
                self.cache_state(None, None)
                return False
            pedal_unhealthy = self.listener is not None and not self.listener.input_healthy
            if pedal_unhealthy:
                if self.control.state is not self.states.FAULT:
                    raise RuntimeError(self.listener.fault_reason or 'pedal input unhealthy')
            elif self.listener:
                for _ in self.listener.take_presses():
                    self.pedal()
            if stop:
                if self.runner:
                    self.runner.status, self.runner.reason = 'STOPPED', 'operator stop'
                self.stop('operator stop')
            feedback = self.read_feedback()
            if feedback is None and getattr(self, 'native_faults', None):
                self.cache_state(None, self.cache.get('joint_limits'))
                return True
            self.current_feedback = feedback
            speed_check=getattr(self,'check_speed_protection',None)
            if speed_check is not None:speed_check(feedback)
            limits = self.backend.get_bimanual_joint_position_limits()
            if hasattr(limits, 'tolist'):
                limits = limits.tolist()
            corrections=[]
            guard=getattr(self,'control_guard',None)
            position_decision=None
            if guard is not None and self.runner is not None:
                position_decision=guard.evaluate(dict(rule_id='G20',signal_id='measured_joint_positions',
                    observed=[feedback['qpos'][i]for i in active_indices('both')],source_time=feedback['sample_time_s'],
                    checked_time=self.control_time(),epoch=self.epoch),'feedback')
            self.position_guard_decision=position_decision
            bounded=observe_arm_positions(feedback['qpos'],limits,environment=self.position_environment,corrections=corrections)
            if position_decision is not None and position_decision['verdict']=='reject':
                raise ValueError('G20: feedback exceeds the bound device capability')
            feedback['position_boundary']=dict(environment=self.position_environment,
                position_noise_rad=position_noise_rad(self.position_environment),
                feedback_margin_rad=feedback_position_margin_rad(self.position_environment),
                command_projection_allowed=all(abs(row['delta_rad'])<=position_noise_rad(self.position_environment)+1e-12 for row in corrections),
                corrections=corrections,
                command_joint_positions_rad=[bounded[i] for i in active_indices('both')])
            if position_decision is not None:feedback['position_boundary']['guard_decision']=position_decision
            if feedback.get('native_faults'):
                self.native_fault(feedback['native_faults'])
                self.cache_state(feedback,limits)
                return True
            if pedal_unhealthy:
                # Keep real feedback/heartbeat after the fault requested damping.
                # An unavailable pedal cannot admit or resume any queued motion.
                self.cache_state(feedback,limits)
                return True
            capture_ack = None
            while not self.commands.empty():
                action, epoch, review = self.commands.get_nowait()
                if action == 'capture_ack':
                    if epoch == self.epoch:capture_ack = review
                    continue
                if action == 'pedal':
                    self.pedal()
                    continue
                if action == 'recover':
                    with self.lock:
                        if (epoch != self.epoch or review != self.pedal_snapshot() or not self.recovery_ready()
                                or not self.recovery_feedback_valid(feedback,limits)):
                            self.event('recover_rejected', reason='owner/feedback/pedal changed')
                            continue
                        reason = self.recoverable_fault or self.control.reason
                        self.invalidate()
                        self.runner = None
                        self.recoverable_fault = None
                        self.recovery_feedback_limit_s = FEEDBACK_TIMEOUT_S
                        self.control.state = self.states.STOPPED
                        self.control.auto_aborted = True
                        self.control.reason = 'motion fault acknowledged; preview a new action'
                        self.event('fault_acknowledged', reason=reason, epoch=self.epoch)
                    continue
                if action in('normal_mode','normal_finish'):
                    if (epoch!=self.epoch or self.control.state is self.states.FAULT
                            or not getattr(self.runner,'is_inference',False)):
                        if getattr(self.runner,'is_inference',False):self.runner.reject_mode(review['request_id'],'owner epoch/state changed')
                        continue
                    try:
                        if action=='normal_finish':self.runner.start_finish(review,feedback)
                        else:self.runner.start_mode(review,feedback)
                    except Exception as error:
                        self.runner.reject_mode(review['request_id'],str(error))
                        raise
                    continue
                review, review_deadline = review
                plans, _, press, _, samples = review
                plan = plans[0]
                accepted_age = 30. + time.monotonic() - review_deadline
                if not 0. <= accepted_age <= 30.:
                    self.reject_confirmation(plans, 'review_expired', review_deadline=review_deadline)
                    continue
                if (epoch != self.epoch or not self.feedback_healthy
                        or self.control.state not in (self.states.READY, self.states.STOPPED, self.states.MANUAL)
                        or self.runner and self.runner.status not in ('READY', 'HOLD_UNSETTLED', 'STOPPED', 'FAULT')
                        or tuple(map(tuple, limits)) != plan.joint_limits
                        or self.pedal_snapshot() != (press, False, True)):
                    self.reject_confirmation(plans, 'owner/pedal/state changed')
                    continue
                if not self.accept_review(plans):
                    self.reject_confirmation(plans, 'review revoked before execution')
                    continue
                self.active_epoch, self.review_press = epoch, press
                self.sequence_remaining = list(plans[1:])
                self.sequence_samples = list(samples[1:]);self.capture_sample = samples[0];self.capture_wait = None;self.capture_stable_since = None
                self.capture_recovery_deadline = None;self.last_capture_ack = None
                self.sequence_deadline = self.control_time() + sum(p.duration_s for p in plans) + 4 * len(plans) + 11*sum(s is not None for s in samples)
                # TTL is checked at the Owner acceptance boundary, before any
                # runner/gain/recording work. Construction cannot turn this
                # accepted review into a hardware fault by crossing the deadline.
                self.accepted_review_age_s = accepted_age
                try:self.start_motion(plan, feedback)
                finally:self.accepted_review_age_s = None
                self.event('motion_confirmed', plan=asdict(plan), status=self.runner.status,
                           execution_settings=self.motion_error_settings())
            if self.runner and self.runner.status not in ('ARMED', 'STOPPED', 'FAULT'):
                # begin/send check the live press counter immediately before writes.
                before = self.runner.status
                self.runner.tick(self.control_time(), feedback)
                if before != self.runner.status:
                    self.event('motion_state', status=self.runner.status, reason=self.runner.reason,
                               motion_record=self.runner.motion_record() if isinstance(self.runner,MotionRunner) and self.runner.status=='READY' else None)
            self.record_arrival(feedback)
            self.diagnostic_history.append(self.diagnostic_sample(feedback))
            if self.capture_sample and self.runner:
                now = self.control_time()
                if self.capture_wait or self.capture_recovery_deadline is not None:
                    self.command_gate()
                    if self.capture_recovery_deadline is not None and now >= self.capture_recovery_deadline:
                        guard_reasons.append('capture_recovery_timeout')
                    if self.capture_wait and now >= self.capture_wait['deadline']:guard_reasons.append('capture_ack_timeout')
                    if not self.feedback_healthy:guard_reasons.append('feedback_unhealthy')
                    target = self.runner.execution_target
                    from arx_env.joint_head_real_motion import command_gap
                    settings = self.motion_error_settings()
                    if settings['arrival_enabled'] and (target is None or any(abs(feedback['qpos'][i]-target[i]) > tolerance
                            for i,tolerance in zip(active_indices('both'), settings['arrival_tolerances_rad']))
                            or any(abs(gap-command_gap(side,target[i])) > settings['arrival_jaw_mm']
                                   for gap,side,i in zip(feedback_gaps(feedback['qpos']),('left','right'),(6,13))
                                   if i in self.runner.enabled_jaws)):
                        guard_reasons.append('target_outside_arrival_tolerance')
                    if self.runner.status not in ('READY','HOLD_UNSETTLED'):guard_reasons.append('motion_not_ready')
                    if guard_reasons:
                        raise ValueError('capture guard: '+', '.join(guard_reasons))
                    if self.capture_wait and (not self.stationary or self.runner.status != 'READY'):
                        if self.capture_recovery_deadline is None:self.capture_recovery_deadline = now+2.
                        self.event('capture_window_revoked',sample_id=self.capture_sample,diagnostic=self.diagnostic_sample(feedback))
                        self.capture_wait = None
                        self.capture_stable_since = None
                if capture_ack is not None:
                    if (self.capture_wait and capture_ack == dict(sample_id=self.capture_sample,
                            capture_ready_wall_ns=self.capture_wait['ready_wall_ns'])):
                        self.event('capture_acknowledged',sample_id=self.capture_sample,diagnostic=self.diagnostic_sample(feedback))
                        self.last_capture_ack = dict(capture_ack)
                        if getattr(self,'environment',None)=='fake':
                            self.last_capture_ack.update(command_epoch=self.epoch,
                                clock_domain=self.capture_wait['clock_domain'],source_clock_id=self.capture_wait['source_clock_id'],
                                capture_ready_source_ns=self.capture_wait['ready_source_ns'])
                        self.capture_sample = None;self.capture_wait = None
                        self.capture_recovery_deadline = None
                    else:
                        self.event('capture_ack_rejected',ack=capture_ack,reason='capture window revoked or changed')
                if self.capture_sample and not self.capture_wait:
                    if self.runner.status == 'READY' and self.feedback_healthy and self.stationary:
                        if self.capture_stable_since is None:self.capture_stable_since = now
                        if now-self.capture_stable_since >= .5:
                            self.last_capture_window_ns = max(time.time_ns(),self.last_capture_window_ns+1)
                            self.capture_wait = dict(sample_id=self.capture_sample,deadline=now+10.,ready_wall_ns=self.last_capture_window_ns,ack_queued=False)
                            if getattr(self,'environment',None)=='fake':
                                from robot_workbench.sample_clock import clock_spec, sample_stamp
                                clock=self.feedback_clock or clock_spec('fake',clock_domain=feedback.get('clock_domain'),
                                                                       source_clock_id=feedback.get('source_clock_id'))
                                source_ns,step=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
                                self.capture_wait.update(ready_source_ns=source_ns,ready_physics_step=step,
                                                         clock_domain='simulation',source_clock_id=clock['source_clock_id'])
                            self.event('awaiting_capture',sample_id=self.capture_sample,diagnostic=self.diagnostic_sample(feedback))
                    else:self.capture_stable_since = None
            if self.sequence_remaining and self.runner and self.runner.status == 'READY' and self.capture_sample is None:
                next_plan = self.sequence_remaining[0]
                actual_gap=feedback_gaps(feedback['qpos'])
                from arx_env.joint_head_jaw_mapping import command_gap_mm
                target_gap=[command_gap_mm(side,next_plan.q_start[i]) for side,i in [('left',6),('right',13)]]
                settings = self.motion_error_settings()
                if settings['arrival_enabled'] and (any(abs(feedback['qpos'][i]-next_plan.q_start[i]) > tolerance for i,tolerance in zip(active_indices('both'), settings['arrival_tolerances_rad']))
                        or any(abs(a-b)>settings['arrival_jaw_mm'] for side,a,b in zip(('left','right'),actual_gap,target_gap)
                               if side not in getattr(self.backend, 'passive_jaws', ()))):
                    raise ValueError('waypoint outside arrival tolerance')
                self.command_gate()
                self.sequence_remaining.pop(0)
                self.capture_sample = self.sequence_samples.pop(0)
                self.capture_stable_since = None
                self.capture_recovery_deadline = None
                self.start_motion(next_plan, feedback, reviewed_deadline=self.sequence_deadline)
            self.cache_state(feedback, limits)
        except Exception as error:
            if self.cancel_requested_reference(error):
                self.cache_state(feedback,self.cache.get('joint_limits'))
                return self.control.state is not self.states.RELEASED
            # Freeze identity and the triggering tick BEFORE epoch invalidation or damping.
            first_fault = self.control.state is not self.states.FAULT
            diagnostic = self.diagnostic_sample(feedback) if first_fault else None
            if diagnostic is not None:
                diagnostic.update(reasons=['joint_speed_peak'] if isinstance(error,JointSpeedProtectionError) else
                    ['joint_position_boundary'] if isinstance(error,JointPositionBoundaryError) else guard_reasons or ['owner_exception'],error=str(error))
            boundary=isinstance(error,JointPositionBoundaryError) and error.details['context']=='feedback'
            if boundary and first_fault:
                self.position_boundary_fault=dict(error.details,reason=str(error),stop_hold_return_confirmed=False,
                    recovery='explicit_ack_after_feedback_inside_observation_envelope_no_resume',
                    guard_decision=getattr(self,'position_guard_decision',None))
                diagnostic['position_boundary_violation']=dict(error.details)
            self.recoverable_fault = (str(error) if isinstance(error,JointSpeedProtectionError) else
                self.position_boundary_fault['reason'] if boundary and self.position_boundary_fault
                and self.position_boundary_fault['reason']==self.control.reason else None)
            if diagnostic is not None and isinstance(error,JointSpeedProtectionError):
                diagnostic['speed_protection']=deepcopy(error.observation)
            if first_fault:self.invalidate()
            if self.runner:
                self.runner.status, self.runner.reason = 'FAULT', str(error)
                self.record_arrival(feedback)
            if self.control.state is not self.states.FAULT:
                try:
                    self.control.fault(str(error))
                    if boundary:
                        self.position_boundary_fault.update(stop_hold_return_confirmed=True,stop_return_control_s=self.control_time())
                        self.recoverable_fault=self.position_boundary_fault['reason']
                    if isinstance(error,JointSpeedProtectionError):
                        self.speed_protection_trip['stop'].update(hold_return_confirmed=True,hold_return_wall_ns=time.time_ns())
                except Exception as stop_error:
                    self.recoverable_fault = None
                    self.control.reason += f'; damping failed: {stop_error}'
                    if isinstance(error,JointSpeedProtectionError):self.speed_protection_trip['stop']['error']=str(stop_error)
            if diagnostic is not None:
                self.last_fault_diagnostic = diagnostic
                self.event('fault_diagnostic',diagnostic=diagnostic,
                           history=list(self.diagnostic_history),history_capacity=200,
                           epoch_after=self.epoch)
            if first_fault:self.event('fault', reason=self.control.reason)
            with self.lock:
                self.cache.update(state=self.control.state.value, reason=self.control.reason,
                                  automatic_enabled=False, feedback_healthy=self.feedback_healthy if isinstance(error,JointSpeedProtectionError) else False, stationary=False,
                                  command_epoch=self.epoch,last_fault_diagnostic=self.last_fault_diagnostic,
                                  recoverable_fault=self.recoverable_fault)
                if self.runner:self.cache['motion']={'status':self.runner.status,'reason':self.runner.reason,'sample_id':None}
        return self.control.state is not self.states.RELEASED

    def accept_review(self, plans):
        """Embedding runtimes may atomically revoke queued, not-yet-started reviews."""
        return True

    def reject_confirmation(self, plans, reason, *, review_deadline=None):
        for plan in plans:
            self.consumed.add(plan.sha256)
            for name in ('frozen_payloads', 'alignment_plans'):
                getattr(self, name, {}).pop(plan.sha256, None)
        self.last_confirm_rejection = dict(id=secrets.token_hex(12),owner_pid=os.getpid(),
            command_epoch=self.epoch,plan_sha256=[p.sha256 for p in plans],reason=reason,
            motion_started=False,monotonic_s=time.monotonic(),wall_ns=time.time_ns(),
            review_deadline_monotonic_s=review_deadline)
        self.event('confirm_rejected', **self.last_confirm_rejection)

    def start_motion(self, plan, feedback, *, reviewed_deadline=None):
        previous=getattr(getattr(self,'runner',None),'target_interpreter',None)
        reuse=bool(previous is not None and not previous.cancelled and previous.epoch==self.epoch
            and previous.active_arm==plan.active_arm
            and (previous.feedforward is None or self.shared_target_profile!='normal_X5')
            and previous.profile==self.shared_target_profile)
        def begin(measured):
            return tuple(measured)if reuse else self.begin(plan.active_arm,measured)
        def command_start():
            if not reuse:
                value=self.last_begin_command
                if not self.preserve_begin_jaws:return None
                if not value or value['epoch']!=self.epoch:raise ValueError('current-epoch strict begin command missing')
                return value['command_sdk14'][:]
            return previous.reference.sample([self.control_time()-previous.start_s])[0][0].tolist()
        self.plan_sequence += 1
        epoch = self.active_epoch
        self.arrival_pending = dict(event_id=f'{os.getpid()}:{self.arrival_nonce}:{epoch}:{self.plan_sequence}',
                                    owner_pid=os.getpid(), command_epoch=epoch,
                                    plan_sha256=plan.sha256, sample_id=self.capture_sample) if plan.kind == 'workbench' else None
        self.runner = MotionRunner(plan, begin=begin,
                                   send=lambda q: self.send(plan.active_arm, q),
                                   stop=self.motion_fault, consumed=self.consumed, arm_tolerance=self.arm_tolerance, arm_tolerances=self.arm_tolerances,
                                   environment=self.position_environment,
                                   execution_settings=getattr(self, 'execution_error_settings', None),
                                   guard=getattr(self,'control_guard',None),
                                   record_event=getattr(self,'record_control_event',None),
                                   disabled_grippers=getattr(self.backend, 'passive_jaws', ()),
                                   interpreter_factory=(lambda reference,side,stamp:self.make_target_interpreter(reference,side,stamp,previous=previous))if self.shared_target_profile else None,
                                   command_start_factory=command_start if reuse or self.preserve_begin_jaws else None)
        self.runner.start(self.control_time(), feedback, reviewed_deadline=reviewed_deadline,
                          review_age_s=self.review_age())

    @property
    def shared_target_profile(self):
        return getattr(self.backend,'target_interpreter_profile',None)

    def _bind_reference_context(self, policy=None, velocity_source=None):
        from robot_workbench.execution_settings import reference_execution_limits
        from robot_workbench.sdk_process import EXECUTION_CONTEXT_SCHEMA
        if getattr(self.backend, 'execution_context_capabilities', None) != {'schema': EXECUTION_CONTEXT_SCHEMA}:
            if policy is not None:
                raise ReferenceWindowRejected('SDK worker lacks bound execution context capability')
            return
        context = dict(controller_profile=self.shared_target_profile,
                       jaw_calibration_id=getattr(self.backend, 'native_gripper_calibration_id', None))
        if policy is not None: context['limits'] = reference_execution_limits(policy)
        if velocity_source == 'sdk_position_only': context['velocity_source'] = velocity_source
        self.command_gate()
        self.backend.bind_execution_context(context)
        self.command_gate()

    def make_target_interpreter(self,reference,active_arm,start_s,*,integral=None,feedforward=None,previous=None,integral_gains=None,record_event=None):
        from robot_workbench.target_interpreter import TargetInterpreter
        profile=self.shared_target_profile
        if profile is None:raise ValueError('shared target interpreter is not configured')
        # Reference timing and native gains can be selected independently.
        if integral is None:integral=getattr(self.backend,'motion_gain_profile',profile)!='normal_X5'
        policy = reference.metadata.get('planning_policy')
        config = getattr(self.runner, 'config', {})
        run_policy = config.get('reference_planning')
        if policy is not None and (not getattr(self.runner, 'is_inference', False) or policy != run_policy
                                   or self.runner._epoch != self.epoch):
            raise ReferenceWindowRejected('reference planning policy differs from current reviewed run')
        if getattr(self.runner, 'is_inference', False) and run_policy is not None:
            if self.runner._epoch != self.epoch:
                raise ReferenceWindowRejected('execution context belongs to a revoked run')
            policy = run_policy
        self._bind_reference_context(policy, reference.metadata.get('velocity_source'))
        interpreter=TargetInterpreter(self.backend,reference,active_arm=active_arm,start_s=start_s,
            epoch=self.epoch,current_epoch=lambda:self.epoch,gate=self.command_gate,clock=self.control_time,
            profile=profile,integral=integral,
            feedforward=feedforward,integral_gains=integral_gains,guard=getattr(self,'control_guard',None),record_event=record_event)
        interpreter.inherit_hold(previous,self.control_time())
        if reference.metadata.get('kind')=='point'and self.sequence_deadline is not None:
            self.sequence_deadline+=max(0.,reference.duration_s-reference.metadata['source_duration_s'])
        self.event('target_interpreter_bound',reference=reference.metadata,active_arm=active_arm,
            controller_profile=profile,command_epoch=self.epoch,source_time_origin_s=start_s)
        return interpreter

    def motion_error_settings(self):
        from robot_workbench.execution_settings import defaults
        return getattr(self, 'execution_error_settings', None) or defaults('calibration',
            arrival_tolerances(self.arm_tolerance, self.arm_tolerances))

    def record_arrival(self, feedback):
        if self.runner and self.runner.status in ('STOPPED', 'FAULT'):
            self.arrival_pending = None
        if not self.arrival_pending or not self.runner or self.runner.status != 'READY' or feedback is None:
            return
        target = self.runner.execution_target if self.runner.execution_target is not None else self.runner.plan.q_target
        event = dict(self.arrival_pending, status='success',
                     arrival_enforced=self.motion_error_settings()['arrival_enabled'],
                     execution_settings=self.motion_error_settings(),
                     q_target_rad=[float(q) for i,q in enumerate(target) if i not in (6,13)],
                     q_actual_rad=[float(q) for i,q in enumerate(feedback['qpos']) if i not in (6,13)],
                     timestamp_ns=time.time_ns(), reason=self.runner.reason)
        self.arrival_pending = None
        with self.lock:
            self.arrival_events.append(event)
            self.cache['arrival_events'] = list(self.arrival_events)
        self.event('arrival', **event)

    def cache_state(self, feedback, limits):
        press, down, healthy = self.pedal_snapshot()
        now=time.monotonic();gains_key=(self.epoch,self.control.state.value,self.backend.is_connected)
        if self.backend.is_connected and (gains_key!=self.local_gains_key or now-self.local_gains_checked>=1.):
            self.local_gains_cache=self.backend.read_gains()
            self.local_gains_checked=now;self.local_gains_wall_ns=time.time_ns();self.local_gains_key=gains_key
        value = dict(pid=os.getpid(), mode='hardware' if self.hardware else 'simulation',
                     arrival_events=list(self.arrival_events),
                     capture_diagnostics_schema='joint-head-capture-diagnostic-v1',
                     evidence_logging_healthy=self.evidence_logging_healthy,
                     stationary_diagnostic=getattr(self.stationary_window,'last_diagnostic',None),
                     last_fault_diagnostic=self.last_fault_diagnostic,
                    position_boundary_fault=deepcopy(self.position_boundary_fault),
                     last_capture_ack=self.last_capture_ack,
                     last_confirm_rejection=self.last_confirm_rejection,
                     simulation_only=not self.hardware,
                     jaw_mapping=mapping_identity(),
                     feedback_limits=dict(joint_position_margin_rad=feedback_position_margin_rad(self.position_environment),
                                          jaw_range_mm=[min(feedback_min_mm(s)for s in ('left','right')),max(feedback_max_mm(s)for s in ('left','right'))],
                                          jaw_ranges_mm={s:[feedback_min_mm(s),feedback_max_mm(s)]for s in ('left','right')},command_limits_unchanged=True),
                     evidence_source='physical_hardware' if self.hardware else 'simulated_owner',
                     control_clock='wall_monotonic' if self.control_clock is None else 'injected',
                     motion_protocol='reviewed-single-arm-v1',
                     passive_jaws=list(getattr(self.backend, 'passive_jaws', ())),
                     stop_gain_profile={"joint_kp":[24.,21.,21.,9.,9.,6.],"joint_kd":[1.2,1.2,1.2,1.1,1.1,1.07],"gripper_kp":.12,"gripper_kd":.15}, fault_manual=getattr(self,'fault_manual',False),
                     capabilities=['single_arm_home', 'both_home_simultaneous', 'joint_speed_0.2', 'workbench_trajectory_v1', 'calibration_capture_ack_v1', 'calibration_capture_window_v2'],
                     state=self.control.state.value, reason=self.control.reason,
                     recoverable_fault=self.recoverable_fault,
                     connected=self.backend.is_connected, command_epoch=self.epoch,
                     automatic_enabled=bool(self.capture_sample or self.runner and self.runner.status in ('VERIFYING', 'RUNNING', 'SETTLING')),
                     receive_ns=time.time_ns(), calibration_feedback=feedback, joint_limits=limits,
                     feedback_healthy=self.feedback_healthy,
                     arm_arrival_tolerance_rad=self.arm_tolerance if self.arm_tolerances is None and not getattr(self,'execution_error_settings',None) else None,
                     arm_arrival_tolerances_rad=list(self.motion_error_settings()['arrival_tolerances_rad']),
                     position_noise_rad=position_noise_rad(self.position_environment),position_environment=self.position_environment,
                     stationary=self.stationary,
                     local_gains=self.local_gains_cache if self.backend.is_connected else None,
                     local_gains_sample_wall_ns=self.local_gains_wall_ns,
                     release_requires='ARMS_SUPPORTED_RELEASE', stop_damping_scale=self.stop_scale,
                     pedal=dict(input_healthy=healthy, physically_down=down, last_press_ns=press,
                                stop_key_code=getattr(self.listener,'key_code',None),
                                latched=bool(self.listener and self.listener.requested.is_set())),
                     motion=None if self.runner is None else dict(status='AWAITING_CAPTURE' if self.capture_wait else self.runner.status, reason=self.runner.reason,
                                                                 sample_id=self.capture_sample,
                                                                 capture_recovery_deadline_monotonic_s=self.capture_recovery_deadline,
                                                                 capture_deadline_monotonic_s=self.capture_wait['deadline'] if self.capture_wait else None,
                                                                 capture_ready_wall_ns=self.capture_wait['ready_wall_ns'] if self.capture_wait else None,
                                                                 capture_ready_source_ns=self.capture_wait.get('ready_source_ns') if self.capture_wait else None,
                                                                 capture_ready_physics_step=self.capture_wait.get('ready_physics_step') if self.capture_wait else None,
                                                                 capture_clock_domain=self.capture_wait.get('clock_domain') if self.capture_wait else None,
                                                                 capture_source_clock_id=self.capture_wait.get('source_clock_id') if self.capture_wait else None,
                                                                 plan_sha256=self.runner.plan.sha256,
                                                                 active_arm=self.runner.plan.active_arm,
                                                                 position_boundary=dict(environment=self.runner.plan.environment,
                                                                     plan_corrections=self.runner.plan.position_corrections,
                                                                     execution_start_corrections=self.runner.execution_start_corrections),
                                                                 target_error=self.runner.target_error),
                     sdk_stop_limit='Software stop cannot interrupt a blocked native SDK call')
        with self.lock:
            # Publish state and its command generation together. A Stop/recover
            # after acquisition must not make the old feedback look current.
            from robot_workbench.state_contract import fact
            self.state_revision=getattr(self,'state_revision',0)+1
            feedback_epoch=(feedback or {}).get('command_epoch',self.epoch)
            value.update(state=self.control.state.value,reason=self.control.reason,
                recoverable_fault=self.recoverable_fault,command_epoch=self.epoch)
            value['control_publication']=fact('control','Runtime.Owner',getattr(self,'runtime_nonce',None),
                self.state_revision,(feedback or {}).get('sample_wall_ns'),
                dependencies=dict(command_epoch=self.epoch,feedback_epoch=feedback_epoch),
                valid=feedback_epoch==self.epoch,reason=None if feedback_epoch==self.epoch else 'feedback_precedes_control_transition')
            self.cache = value


def handler(owner):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def valid_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.valid_host():
                return self.reply(403, {'error': 'loopback Host required'})
            self.reply(200 if self.path == '/state' else 404, owner.snapshot() if self.path == '/state' else {})

        def do_POST(self):
            if (self.path != '/operator' or not self.valid_host()
                    or self.headers.get('Origin') != 'http://' + self.headers.get('Host', '')):
                return self.reply(403, {'error': 'same-origin operator command required'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4*1024*1024 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('small JSON command required')
                self.connection.settimeout(.5)
                result = owner.submit(json.loads(self.rfile.read(length)))
                self.reply(202, result)
            except (ValueError, TypeError, queue.Full) as error:
                self.reply(400, {'error': str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-source', type=Path, required=True)
    parser.add_argument('--connect-hardware', action='store_true')
    parser.add_argument('--passive-left-jaw', action='store_true', help='Disable left jaw position/velocity gains; raw feedback remains invalid')
    arrival = parser.add_mutually_exclusive_group()
    arrival.add_argument('--arm-tolerance', type=float, choices=(.01, .02, DEFAULT_JOINT_TOLERANCE_RAD, .05), default=DEFAULT_JOINT_TOLERANCE_RAD)
    arrival.add_argument('--arm-tolerances', type=float, nargs=12, metavar='RAD', help='Left J1..J6 then right J1..J6 arrival tolerances')
    parser.add_argument('--port', type=int, default=8098)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--jaw-mapping-file', type=Path)
    parser.add_argument('--simulation-state', type=Path, help='Simulation-only initial physical14 JSON')
    parser.add_argument('--stop-damping-scale', type=float, choices=(1., 1.8), default=1.)
    args = parser.parse_args()
    if args.jaw_mapping_file:
        from arx_env.joint_head_jaw_mapping import configure_mapping
        configure_mapping(args.jaw_mapping_file)
    try:arrival_tolerances(args.arm_tolerance,args.arm_tolerances)
    except (TypeError,ValueError) as exc:parser.error(str(exc))
    if not 1024 <= args.port <= 65535:
        parser.error('unprivileged loopback port required')
    if args.connect_hardware and args.simulation_state:
        parser.error('simulation state cannot initialize hardware')
    simulation_q = None
    if args.simulation_state:
        simulation_q = json.loads(args.simulation_state.read_text())
        if (not isinstance(simulation_q, list) or len(simulation_q) != 14
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in simulation_q)):
            parser.error('simulation state must contain 14 finite numbers')
    safety = legacy_modules(args.legacy_source)
    args.output.mkdir(parents=True, exist_ok=False)
    # Bounded asynchronous evidence output cannot block physical-pedal processing.
    logs = queue.Queue(maxsize=2048)
    log_failed = threading.Event()
    def event(kind, **fields):
        try:
            logs.put_nowait(dict(time_ns=time.time_ns(), event=kind, **fields))
        except queue.Full:
            log_failed.set()
    def log_worker():
        try:
            with (args.output / 'events.jsonl').open('x') as stream:
                while True:
                    item = logs.get()
                    try:
                        if item is None:
                            return
                        stream.write(json.dumps(item, allow_nan=False)+'\n')
                        stream.flush()
                        if item.get('event')=='fault_diagnostic':os.fsync(stream.fileno())
                    finally:
                        logs.task_done()
        except Exception:
            log_failed.set()
    threading.Thread(target=log_worker, daemon=True).start()
    listener = None
    backend = hardware_backend(passive_left_jaw=args.passive_left_jaw) if args.connect_hardware else SimulatedBackend(simulation_q)
    if args.connect_hardware:
        from arx_env.pedal_stop import PedalStopListener
        listener = PedalStopListener()
        listener.start()
    owner = Owner(backend, safety, listener=listener, hardware=args.connect_hardware,
                  stop_scale=args.stop_damping_scale, event=event)
    owner.arm_tolerance = args.arm_tolerance
    owner.arm_tolerances = args.arm_tolerances
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(owner))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    def stop_signal(*_):
        owner.submit({'action': 'stop'})
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_signal)
    try:
        if args.connect_hardware:
            backend.connect()
            backend.set_damping_hold(manual=True)
    except Exception as error:
        owner.control.state = owner.states.FAULT
        owner.control.reason = f'initialization failed: {error}'
    event('started', pid=os.getpid(), hardware=args.connect_hardware, legacy_source=str(args.legacy_source))
    last_evidence = 0.
    while True:
        owner.evidence_logging_healthy = not log_failed.is_set()
        if log_failed.is_set():
            with owner.lock:owner.cache['logging_warning']='evidence logger unavailable; motion unaffected'
        if not owner.tick():
            break
        now = time.monotonic()
        if now-last_evidence >= .1:
            event('state', **owner.snapshot())
            last_evidence = now
        time.sleep(.01)
    if listener:
        listener.close()
    server.shutdown()
    server.server_close()
    event('released', pid=os.getpid())
    if not log_failed.is_set():
        logs.put(None)
        logs.join()


if __name__ == '__main__':
    main()
