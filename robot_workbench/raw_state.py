"""Extend the existing per-arm feedback read; no controllers or output commands."""
import math
import time
import numpy as np


def _optional(values):
    return [float(x)if x is not None and math.isfinite(float(x))else None for x in values]


def read_snapshot(controllers,sides):
    started=time.monotonic();wall=time.time_ns()
    result=dict(qpos=[],qvel=[],sdk_timestamps=[],sample_time_s=started,sample_wall_ns=wall)
    raw=dict(schema='workbench.raw-state.v1',arms={},
        joint_units='rad',jaw_units='SDK_feedback_m',effort_units='original_SDK_feedback_units',
        eef_units='native_FK_xyz_m_rpy_rad; xyz_wxyz',
        pairing='sequential_per_arm_reads; independent_sdk_clocks; not_simultaneous',
        host_time_scope='getter_receipt_bracket; not_sensor_acquisition_wall_time')
    for side in sides:
        controller=controllers[side]
        before=time.monotonic_ns();before_wall=time.time_ns()
        state=controller.get_joint_state()
        after=time.monotonic_ns();after_wall=time.time_ns()
        q=[*map(float,state.pos()),float(state.gripper_pos)]
        v=[*map(float,state.vel()),float(state.gripper_vel)]
        stamp=float(state.timestamp)
        # Preserve the exact original control feedback validity requirement.
        if len(q)!=7 or len(v)!=7 or not all(math.isfinite(x)for x in[*q,*v,stamp]):
            raise ValueError('nonfinite calibration feedback')
        result['qpos'].extend(q);result['qvel'].extend(v);result['sdk_timestamps'].append(stamp)
        effort=_optional([*state.torque(),getattr(state,'gripper_torque',None)])
        if len(effort)!=7:raise ValueError('Raw arm effort shape differs')
        pose=[None]*6;wxyz=[None]*7;reason=None
        try:
            # The same unrounded q as this feedback, never a second state getter.
            pose=_optional(controller.forward_kinematics(np.asarray(q[:6],dtype=np.float64)))
            if len(pose)!=6 or any(x is None for x in pose):raise ValueError('invalid native FK output')
            roll,pitch,yaw=pose[3:]
            cr,cp,cy=(math.cos(x/2)for x in(roll,pitch,yaw));sr,sp,sy=(math.sin(x/2)for x in(roll,pitch,yaw))
            quat=[cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]
            norm=math.sqrt(sum(x*x for x in quat));wxyz=pose[:3]+[x/norm for x in quat]
        except Exception as error:
            pose=[None]*6;reason=str(error)[:256]
        raw['arms'][side]=dict(qpos=q,qvel=v,effort=effort,eef_pose_6d=pose,eef_wxyz=wxyz,
            sdk_feedback_time_s=stamp,host_read_monotonic_ns=[before,after],host_read_wall_ns=[before_wall,after_wall],
            validity=dict(qpos=True,qvel=True,effort=[x is not None for x in effort],eef=reason is None),
            unavailable=dict(effort='missing_or_nonfinite_original_feedback'if any(x is None for x in effort)else None,eef=reason))
    raw['status']='observed'if all(row['validity']['eef']for row in raw['arms'].values())else'degraded'
    result['raw_state']=raw
    return result
