"""ARX6 recorded encoder coordinates, separate from simulation calibration.

Jaw metres name the 10/28 endpoint coordinate used by the recorder. Replacement
fingertips do not make that coordinate a freshly measured physical aperture.
"""
from copy import deepcopy
import hashlib
import json

import numpy as np

from .inference import vector14,native_device_to_model,native_model_to_device,decode_native14
from .inference_model import CAMS
from .inference_owner import digest, device_limits
from .sdk_quiesce import load_gripper_calibration

SCHEMA='workbench.mixed366_native_coordinates.v1'
ENDPOINTS={
    'left':dict(closed_readout=6.217479209582667,open_readout=2.9787518120088503,width_m=.083),
    'right':dict(closed_readout=6.154535744258794,open_readout=2.900930800335699,width_m=.083),
}
SERIALS=dict(head='260522279090',left='260622271169',right='260622271762')


def validate_profile(profile):
    if (not isinstance(profile,dict) or set(profile)!={'schema','robot_id','sdk_module','gripper_calibration','camera_serials','gripper'}
            or profile['schema']!=SCHEMA or profile['robot_id']!='arx6'
            or profile['camera_serials']!=SERIALS or profile['gripper']!={'max_opening_mm':83.}):
        raise ValueError('explicit ARX6 mixed366 native-coordinate binding required')
    from .gravity_calibration import _read
    pin=profile['sdk_module']
    from .control_plan import _read_file
    if (not isinstance(pin,dict)or set(pin)!={'path','sha256'}
            or hashlib.sha256(_read_file(pin['path'],64*1024*1024)).hexdigest()!=pin['sha256']):
        raise ValueError('native inference SDK binary changed')
    native=load_gripper_calibration(profile['gripper_calibration'],robot_id='arx6')
    if native['sides']!=ENDPOINTS:raise ValueError('native jaw endpoints differ from the training recorder')
    return native


def load_binding(pin,deployment):
    from .gravity_calibration import _read
    profile=json.loads(_read(pin));validate_profile(profile)
    if (deployment['robot_id']!=profile['robot_id'] or deployment.get('disabled_grippers')!=[]
            or deployment['sdk_quiesce']['module']!=profile['sdk_module']
            or deployment.get('native_gripper_calibration')!=profile['gripper_calibration']
            or {k:v['serial']for k,v in deployment['cameras'].items()}!=profile['camera_serials']):
        raise ValueError('inference native coordinates differ from the actual SDK/camera deployment')
    return profile,dict(schema=SCHEMA,robot_id=profile['robot_id'],camera_serials=deepcopy(SERIALS),
        calibration_profile_digest=digest(profile),coordinate_qualification='recorded_native_state14',
        extrinsics_consumed=False)


class NativeAdapter:
    def __init__(self,profile,sdk_limits,disabled_grippers=()):
        native=validate_profile(profile)
        from arx_env.joint_head_jaw_mapping import mapping_identity
        if disabled_grippers or mapping_identity().get('native_calibration_id')!=native['identity']:
            raise ValueError('mixed366 needs measured feedback and output for both calibrated jaws')
        self.limits=np.asarray(device_limits(sdk_limits),float)
        self.limits[[6,13],1]=np.minimum(self.limits[[6,13],1],profile['gripper']['max_opening_mm'])

    @staticmethod
    def device_to_model(value):
        return native_device_to_model(value,clip_jaws=True)

    @staticmethod
    def model_to_device(value):
        return native_model_to_device(value)

    def model_input(self,observation):
        images=observation['images']
        if set(images)!=set(CAMS) or any(np.asarray(x).dtype!=np.uint8 or np.asarray(x).shape!=(480,640,3)for x in images.values()):
            raise ValueError('mixed366 requires three raw RGB8 640x480 frames; resize belongs to frozen training preprocessing')
        return self.device_to_model(observation['device_state14']).astype(np.float32),images

    def decode(self,prediction,current_device14):
        return decode_native14(prediction,current_device14,self.limits)


def adapter(profile,sdk_limits,disabled_grippers):
    value=NativeAdapter(profile,sdk_limits,disabled_grippers)
    return value,dict(schema=SCHEMA,profile_digest=digest(profile),joint_offsets_rad=[0.]*12,
        model_jaw_unit='recorder_endpoint_m',device_jaw_unit='recorder_endpoint_mm',
        input_jaw_clipping_m=[0.,.083],command_jaw_limits_mm=[0.,profile['gripper']['max_opening_mm']],arms='both',history_steps=0,
        first_action_offset_s=1/30.,position_projection=True)
