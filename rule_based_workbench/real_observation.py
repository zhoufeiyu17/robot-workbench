"""Seal head-camera physical RGB-D observations for the existing detector.

The caller must check hardware_access before invoking capture(). This module
has no network or controller access; read_snapshot returns one atomic ZIP body.
"""
from copy import deepcopy
from functools import lru_cache
import hashlib
import io
import json
import math
from pathlib import Path
import time
import zipfile

from integration import _digest, validate_profile


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_initial_sample(read_owner, *, wall_ns=time.time_ns):
    """Read-only owner snapshot; same-host wall timestamps also bound transport age."""
    from contract import validate_physical_feedback
    owner = read_owner()
    received = wall_ns()
    feedback = owner.get('calibration_feedback') or {}
    age = owner.get('sample_age_s')
    stamp = feedback.get('sample_wall_ns')
    if (owner.get('mode') != 'hardware' or owner.get('connected') is not True
            or owner.get('feedback_healthy') is not True or owner.get('stationary') is not True
            or owner.get('automatic_enabled') is not False or owner.get('acquisition_pending')
            or owner.get('state') not in ('READY', 'STOPPED_DAMPING_HOLD', 'MANUAL_TAKEOVER')
            or (owner.get('motion') or {}).get('status') in ('VERIFYING', 'RUNNING', 'SETTLING', 'AWAITING_CAPTURE')
            or type(age) not in (float,int) or not 0 <= age <= .1
            or type(stamp) is not int or not 0 <= received-stamp <= 500_000_000):
        raise ValueError('fresh healthy stationary physical owner feedback required')
    state = deepcopy(validate_physical_feedback(owner.get('physical_feedback')))
    evidence = dict(pid=owner.get('pid'), command_epoch=owner.get('command_epoch'),
                    sample_time_s=feedback.get('sample_time_s'), sample_wall_ns=stamp,
                    received_wall_ns=received, sample_age_s=age)
    return state, evidence


def seal_initial_state(before, after, profile):
    from joint_space import convert_state, gripper_offset_mm, validate_initial_state
    measured, evidence = before
    planned = deepcopy(measured)
    for side in ('left', 'right'):
        key = side+'_gripper'
        planned[key] = min(max(planned[key], max(0., gripper_offset_mm(profile))), profile['gripper']['max_opening_mm'])
    value = dict(schema_version='physical_initial_state.v1', calibration_profile_digest=_digest(profile),
                 measured_reported=measured, planned_reported=planned,
                 model=convert_state(planned, profile, to_reported=False),
                 jaw_normalization='clamp_to_model_command_limits_within_2mm',
                 owner_samples=[evidence, after[1]], after_observation_reported=after[0])
    return validate_initial_state(value, profile)


def validate_binding(profile, binding):
    validate_profile(profile)
    serials = binding.get('camera_serials')
    if (binding.get('calibration_accepted') is not True or binding.get('evidence_source') != 'physical_rgbd'
            or not isinstance(binding.get('robot_id'), str) or not binding['robot_id'].strip()
            or not isinstance(serials, dict) or set(serials) != {'head','left','right'}
            or any(not isinstance(v,str) or not v.strip() for v in serials.values())
            or len(set(serials.values())) != 3 or binding.get('calibration_profile_digest') != _digest(profile)):
        raise ValueError('accepted physical calibration, robot and three camera identities required')


def inverse_brown_rectified_pixels(pixels, intr):
    """librealsense 2.56.5 deprojection, as calibration rectify_corners / d405.py.

    Returns rectified pixels at unchanged K; inverse Brown is NOT forward Brown.
    """
    import numpy as np
    pixels = np.asarray(pixels, dtype=np.float64)
    original = (pixels - [intr['cx'], intr['cy']]) / [intr['fx'], intr['fy']]
    x, y = original[..., 0].copy(), original[..., 1].copy()
    k1, k2, p1, p2, k3 = intr['coefficients']
    with np.errstate(over='ignore', divide='ignore', invalid='ignore'):
        for _ in range(10):
            r2 = x*x + y*y
            ir = 1 / (1 + ((k3*r2 + k2)*r2 + k1)*r2)
            xq, yq = x/ir, y/ir
            dx = 2*p1*xq*yq + p2*(r2 + 2*xq*xq)
            dy = 2*p2*xq*yq + p1*(r2 + 2*yq*yq)
            x, y = (original[..., 0]-dx)*ir, (original[..., 1]-dy)*ir
    return np.stack((x,y), axis=-1)*[intr['fx'],intr['fy']]+[intr['cx'],intr['cy']]


@lru_cache(maxsize=3)
def inverse_brown_map(serialized_intrinsics):
    import numpy as np
    intr = json.loads(serialized_intrinsics)
    yy, xx = np.indices((intr['height'], intr['width']), dtype=np.float64)
    target = np.stack((xx, yy), axis=-1)
    raw = target.copy()
    for _ in range(30):
        error = target - inverse_brown_rectified_pixels(raw, intr)
        raw += error
        if np.isfinite(error).all() and np.max(np.abs(error)) < 1e-5:
            break
    residual = inverse_brown_rectified_pixels(raw, intr) - target
    if not np.isfinite(raw).all() or not np.isfinite(residual).all() or np.max(np.abs(residual)) > .001:
        raise ValueError('inverse-Brown image remap failed convergence')
    raw.setflags(write=False)
    return raw


def remap_rgbd(rgb, depth, mapping):
    """RGB bilinear, depth nearest; out-of-image samples zero, zero depth preserved."""
    import numpy as np
    h, w = depth.shape
    x, y = mapping[..., 0], mapping[..., 1]
    inside = (x >= 0) & (x <= w-1) & (y >= 0) & (y <= h-1)
    # Clip before integer conversion: converged coordinates may be outside the image.
    x, y = np.clip(x, 0, w-1), np.clip(y, 0, h-1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0+1, w-1), np.minimum(y0+1, h-1)
    dx, dy = (x-x0)[...,None], (y-y0)[...,None]
    color = ((1-dy)*((1-dx)*rgb[y0,x0]+dx*rgb[y0,x1]) +
             dy*((1-dx)*rgb[y1,x0]+dx*rgb[y1,x1]))
    color = np.rint(color).clip(0,255).astype(np.uint8)
    distance = depth[np.floor(y+.5).astype(int), np.floor(x+.5).astype(int)].astype(np.uint16)
    color[~inside] = 0
    distance[~inside] = 0
    return color, distance


def decode_snapshot(body, serial, profile):
    import numpy as np
    from PIL import Image
    if not isinstance(body, bytes) or len(body) > 32_000_000:
        raise ValueError('bounded atomic snapshot ZIP required')
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        if set(archive.namelist()) != {'metadata.json','color.png','depth.png'} or len(archive.infolist()) != 3:
            raise ValueError('snapshot must contain exactly metadata, color and aligned depth')
        if sum(item.file_size for item in archive.infolist()) > 32_000_000:
            raise ValueError('snapshot exceeds decoded transport size')
        metadata = json.loads(archive.read('metadata.json'))
        color_bytes,depth_bytes=archive.read('color.png'),archive.read('depth.png')
        if (len(color_bytes)<26 or len(depth_bytes)<26 or color_bytes[24:26]!=bytes([8,2])
                or depth_bytes[24:26]!=bytes([16,0])):
            raise ValueError('PNG headers must identify RGB8 and grayscale uint16')
        color_image = Image.open(io.BytesIO(color_bytes))
        depth_image = Image.open(io.BytesIO(depth_bytes))
        if color_image.format != 'PNG' or color_image.mode != 'RGB' or depth_image.format != 'PNG' or depth_image.mode not in ('I;16','I;16B','I'):
            raise ValueError('lossless RGB8 and unsigned 16-bit depth PNG required')
        rgb, depth = np.asarray(color_image), np.asarray(depth_image)
    if (metadata.get('serial') != serial or metadata.get('camera_name') != 'head'
            or metadata.get('ready') is not True or metadata.get('rgbd_paired') is not True
            or metadata.get('raw') is not True or metadata.get('annotated') is not False
            or metadata.get('depth_aligned_to') != 'color' or metadata.get('depth_encoding') != 'uint16_png'
            or metadata.get('timestamp_kind') != 'host_receive' or not metadata.get('pair_id')):
        raise ValueError('snapshot identity or RGB-D pairing metadata invalid')
    stamp = metadata.get('host_receive_ns')
    if isinstance(stamp,str) and stamp.isdigit():
        stamp = int(stamp)
    frame, depth_frame = metadata.get('frame_number'), metadata.get('depth_frame_number')
    if any(type(v) is not int or v < 0 for v in (stamp,frame,depth_frame)) or stamp == 0:
        raise ValueError('positive host timestamp and camera frame counters required')
    numbers=[metadata.get(k) for k in ('device_timestamp_ms','depth_device_timestamp_ms','capture_fps','depth_scale_m')]
    if any(type(v) not in (int,float) or not math.isfinite(v) or v < 0 for v in numbers) or min(numbers[2:]) <= 0:
        raise ValueError('finite camera timing, cadence and positive depth scale required')
    if abs(numbers[0]-numbers[1]) > 1000/numbers[2] or not metadata.get('timestamp_domain'):
        raise ValueError('RGB-D pair exceeds one frame interval')
    intr = metadata.get('intrinsics',{})
    k_values = [intr.get(k) for k in ('fx','fy','cx','cy')]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in k_values) or min(k_values[:2]) <= 0:
        raise ValueError('finite pinhole intrinsics required')
    calibrated = profile.get('physical_calibration', {}).get('intrinsics', {}).get('head')
    if calibrated is not None:
        if (calibrated.get('serial') != serial
                or any(calibrated.get(k) != intr.get(k) for k in ('width', 'height'))
                or any(abs(calibrated[k]-intr[k]) > 1e-6 for k in ('fx','fy','cx','cy'))):
            raise ValueError('live camera intrinsics differ from installed head calibration')
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8
            or depth.shape != rgb.shape[:2] or (intr.get('height'),intr.get('width')) != depth.shape
            or depth.dtype.kind not in 'ui' or depth.min() < 0 or depth.max() > 65535):
        raise ValueError('RGB-D dimensions or unsigned depth values invalid')
    coefficients = intr.get('coefficients')
    if not isinstance(coefficients,list) or any(type(v) not in (int,float) or not math.isfinite(v) for v in coefficients):
        raise ValueError('explicit lens distortion coefficients required')
    original_intrinsics=deepcopy(intr)
    fx,fy,cx,cy=k_values
    K=np.asarray([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=np.float64)
    if intr.get('rectified') is not True and any(abs(v)>1e-12 for v in coefficients):
        model = str(intr.get('distortion_model', '')).split('.')[-1]
        if model not in ('brown_conrady', 'inverse_brown_conrady') or len(coefficients)!=5:
            raise ValueError('unsupported nonzero distortion model; requires reviewed rectification')
        if model == 'inverse_brown_conrady':
            rgb, depth = remap_rgbd(rgb, depth, inverse_brown_map(json.dumps(intr, sort_keys=True)))
        else:
            import cv2
            maps=cv2.initUndistortRectifyMap(K,np.asarray(coefficients,dtype=np.float64),None,K,
                                           (intr['width'],intr['height']),cv2.CV_32FC1)
            rgb=cv2.remap(rgb,*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
            depth=cv2.remap(depth.astype(np.uint16),*maps,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        metadata=deepcopy(metadata)
        metadata['intrinsics']={**intr,'coefficients':[0.0]*5,'distortion_model':'none','rectified':True}
    metadata['original_intrinsics']=original_intrinsics
    metadata['rectification']=str(intr.get('distortion_model','')).split('.')[-1]+'_to_same_K' if metadata['intrinsics']!=original_intrinsics else 'none'

    transform = profile['frames']['environment_from_camera']
    fx,fy,cx,cy=k_values
    arrays=dict(rgb=rgb, depth_m=depth.astype(np.float32)*numbers[3],
                K=np.asarray([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=np.float64),
                position=np.asarray(transform['position'],dtype=np.float64),
                quaternion_wxyz=np.asarray(transform['orientation_wxyz'],dtype=np.float64))
    return metadata,stamp,frame,depth_frame,arrays


def capture(read_snapshot, profile, accepted_binding, output, *, interval_s=1.0,
            timeout_s=30.0, monotonic=time.monotonic, sleep=time.sleep, cancel=None):
    """Read at least nine unique atomic pairs covering >=8s (tracking warmup included); seal only on success.

    Failed captures retain partial NPZ evidence, but never a manifest/receipt.
    The callback is task-owned, hardware-gated at its call site, and returns bytes.
    """
    import numpy as np
    validate_binding(profile,accepted_binding)
    if not 0 < interval_s <= 1 or not 8 < timeout_s <= 120:
        raise ValueError('capture cadence must be <=1s with bounded timeout >8s')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    start=monotonic();rows=[];previous=None;seen=set();intrinsics=None;domain=None;original_intrinsics=None
    while True:
        if cancel is not None and cancel():
            raise InterruptedError('观测阶段已中断')
        if monotonic()-start >= timeout_s:
            raise TimeoutError('physical observation did not produce nine pairs spanning eight seconds')
        meta,stamp,frame,depth_frame,arrays=decode_snapshot(read_snapshot(),accepted_binding['camera_serials']['head'],profile)
        current=(stamp,frame,depth_frame,meta['device_timestamp_ms'],meta['depth_device_timestamp_ms'])
        if meta['pair_id'] in seen or previous and any(a<=b for a,b in zip(current,previous)):
            raise ValueError('camera pair/timestamps repeated or regressed')
        if rows and (meta['intrinsics'] != intrinsics or meta['original_intrinsics'] != original_intrinsics or meta['timestamp_domain'] != domain):
            raise ValueError('camera intrinsics or clock domain changed within observation')
        intrinsics,domain=meta['intrinsics'],meta['timestamp_domain'];original_intrinsics=meta['original_intrinsics'];previous=current;seen.add(meta['pair_id'])
        name=f'rgbd-{len(rows):04d}.npz';np.savez_compressed(output/name,**arrays)
        rows.append(dict(path=name,sha256=sha(output/name),source_stamp_ns=stamp,capture_sequence_id=len(rows),
                         pair_id=meta['pair_id'],frame_number=frame,depth_frame_number=depth_frame,
                         device_timestamp_ms=meta['device_timestamp_ms'],depth_device_timestamp_ms=meta['depth_device_timestamp_ms']))
        if len(rows)>=9 and stamp-rows[0]['source_stamp_ns']>=8_000_000_000:
            break
        sleep(interval_s)
    manifest=dict(frame_id='d405_base_0_color_optical_frame',evidence_source='physical_rgbd',
                  robot_id=accepted_binding['robot_id'],camera_serials=deepcopy(accepted_binding['camera_serials']),
                  source_clock='host_receive_wall_ns',capture_hz=1/interval_s,
                  depth_convention='optical_z_m',camera_pose_convention='world_from_ros_optical',
                  intrinsics=intrinsics,original_intrinsics=original_intrinsics,rectification=meta['rectification'],device_timestamp_domain=domain,frames=rows)
    for name,value in [('profile.json',profile),('rgbd-manifest.json',manifest)]:
        (output/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    binding=deepcopy(accepted_binding)
    binding.update(rgbd_manifest_sha256=sha(output/'rgbd-manifest.json'),
                   frame_sha256={row['path']:row['sha256'] for row in rows})
    for obsolete in ('source_digest','detection_files','detection_summary_digest','observations_digest'):
        binding.pop(obsolete,None)
    (output/'acquisition-binding.json').write_text(json.dumps(binding,indent=2,allow_nan=False)+'\n')
    return binding
