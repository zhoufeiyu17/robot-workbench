"""Current-image circular-rim translation; no table Z or temporal smoothing.

Only explicitly registered upright, white-rim assets use this estimator. Raw
FoundationPose output remains in the receipt; a missing/poor rim invalidates
the frame rather than falling back to another position source.
"""
from copy import deepcopy
from functools import lru_cache
import hashlib
import math
from pathlib import Path

METHOD = 'upright_white_circular_rim.v1'


def validate_spec(value, profile_sha256):
    from scene_assets import _sha
    fields = {'schema', 'method', 'cup_profile_sha256', 'perception_asset_digest',
              'mesh_sha256', 'rim_height_m', 'radius_m', 'max_fit_rms_px', 'max_position_delta_m'}
    if (not isinstance(value, dict) or set(value) != fields
            or value.get('schema') != 'cup_pose_refinement.v1' or value.get('method') != METHOD
            or _sha(value.get('cup_profile_sha256')) != _sha(profile_sha256)):
        raise ValueError('cup rim refinement must bind its method and selected profile')
    for key in ('perception_asset_digest', 'mesh_sha256'):
        _sha(value[key])
    for key, limit in (('rim_height_m', .5), ('radius_m', .15),
                       ('max_fit_rms_px', 1.), ('max_position_delta_m', .03)):
        if type(value[key]) not in (int, float) or not math.isfinite(value[key]) or not 0 < value[key] <= limit:
            raise ValueError('finite bounded cup rim refinement field required: ' + key)
    return value


def mesh_rim(path):
    """Offline visual OBJ feature; collision envelopes are not metrology inputs."""
    import numpy as np
    vertices = np.asarray([[float(v) for v in line.split()[1:4]]
                           for line in Path(path).read_text().splitlines() if line.startswith('v ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError('finite canonical visual mesh vertices required')
    top = float(vertices[:, 2].max()); rings = []
    for height in np.unique(np.round(vertices[vertices[:, 2] > top * .9, 2], 6)):
        points = vertices[np.abs(vertices[:, 2] - height) < .51e-6]
        radius = np.linalg.norm(points[:, :2], axis=1)
        points = points[radius > radius.max() - 1e-5]  # Separate inner and outer lip loops.
        if len(points) < 16:
            continue
        xy = points[:, :2]
        cx, cy, offset = np.linalg.lstsq(np.c_[2 * xy, np.ones(len(xy))],
                                       np.sum(xy * xy, axis=1), rcond=None)[0]
        r = math.sqrt(float(offset + cx * cx + cy * cy))
        residual = np.abs(np.linalg.norm(xy - [cx, cy], axis=1) - r)
        angles = np.sort(np.unique(np.round(np.arctan2(xy[:, 1] - cy, xy[:, 0] - cx), 6)))
        gap = float(np.diff(np.r_[angles, angles[0] + 2 * math.pi]).max())
        if math.hypot(cx, cy) <= 1e-5 and float(residual.max()) < 1e-5 and gap < .3:
            rings.append((r, float(points[:, 2].mean())))
    if not rings:
        raise ValueError('complete circular outer rim in the canonical visual mesh required')
    radius, height = max(rings)
    return dict(rim_height_m=height, radius_m=radius)


@lru_cache(maxsize=1)
def source_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def fit_translation(ellipse, k, camera_position, camera_quaternion_wxyz, initial, spec):
    import numpy as np
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    center = np.asarray(ellipse[0], dtype=float); axes = np.asarray(ellipse[1], dtype=float) / 2
    angle = math.radians(float(ellipse[2]))
    q = np.asarray(camera_quaternion_wxyz, dtype=float)
    k = np.asarray(k, dtype=float); initial = np.asarray(initial, dtype=float)
    position = np.asarray(camera_position, dtype=float)
    if (center.shape != (2,) or axes.shape != (2,) or np.any(axes < 4)
            or k.shape != (3, 3) or position.shape != (3,) or initial.shape != (3,) or q.shape != (4,)
            or not all(np.isfinite(v).all() for v in (center, axes, k, position, initial, q))
            or abs(float(np.linalg.norm(q)) - 1) > .001 or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('finite calibrated camera, rim ellipse and initial translation required')
    rotation = Rotation.from_quat([*q[1:], q[0]]).as_matrix()
    ellipse_rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    theta = np.linspace(0, 2 * math.pi, 96, endpoint=False)
    ring = np.c_[spec['radius_m'] * np.cos(theta), spec['radius_m'] * np.sin(theta),
                 np.full(len(theta), spec['rim_height_m'])]
    def residual(translation):
        camera = (ring + translation - position) @ rotation
        if np.any(camera[:, 2] <= 0):
            raise ValueError('cup rim must remain in front of the camera')
        pixels = camera @ k.T
        local = (pixels[:, :2] / pixels[:, 2:] - center) @ ellipse_rotation
        return (np.sqrt(np.sum((local / axes) ** 2, axis=1)) - 1) * float(axes.mean())
    limit = spec['max_position_delta_m']
    result = least_squares(residual, initial, bounds=(initial - limit, initial + limit),
                           max_nfev=100, xtol=1e-11, ftol=1e-11, gtol=1e-11)
    rms = float(np.sqrt(np.mean(residual(result.x) ** 2)))
    delta = float(np.linalg.norm(result.x - initial))
    if not result.success or rms > spec['max_fit_rms_px'] or delta >= limit:
        raise ValueError(f'cup rim geometry rejected: rms={rms:.3f}px, delta={delta*1000:.2f}mm')
    return dict(world_m=[float(v) for v in result.x],
                camera_m=[float(v) for v in ((result.x - position) @ rotation)],
                fit_rms_px=rms, position_delta_m=delta, iterations=int(result.nfev))


def refine_observation(observation, frame_path, selected_cup):
    spec = validate_spec(selected_cup['pose_refinement'], selected_cup['interaction_profile_sha256'])
    value = deepcopy(observation)
    if not value['pair_valid']:
        return value
    from scene_assets import _sha
    import cv2
    import numpy as np
    cup = next(t for t in value['tracks'] if t['object_type'] == 'cup')
    asset = cup['provenance']['asset']
    if (any(asset.get(k) != selected_cup[k] for k in ('asset_id', 'revision'))
            or _sha(asset.get('digest')) != _sha(spec['perception_asset_digest'])):
        raise ValueError('cup rim input belongs to another detected asset')
    for point in (cup['world']['xyz_m'],cup['xyz_m']):
        if len(point)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in point):
            raise ValueError('finite current cup position required before refinement')
    transform=cup['provenance'].get('canonical_from_simulation_mesh_root')
    if (not isinstance(transform,list) or len(transform)!=16
            or any(type(v) not in (int,float) or not math.isfinite(v)
                   or abs(v-(1. if i%5==0 else 0.))>1e-9 for i,v in enumerate(transform))):
        raise ValueError('cup rim refinement requires the registered canonical visual mesh frame')
    receipt = dict(method=METHOD, source_sha256=source_sha256(), spec=deepcopy(spec),
        raw_world_m=deepcopy(cup['world']['xyz_m']), raw_camera_m=deepcopy(cup['xyz_m']),
        input_sha256=hashlib.sha256(Path(frame_path).read_bytes()).hexdigest(),
        source_stamp_ns=value['source_stamp_ns'], table_z_fixed=False, temporal_filter=False)
    # Missing visual evidence is a frame failure. Configuration/asset mismatches
    # above are fatal acquisition errors, not candidates for a stale fallback.
    try:
        box=cup.get('image_bbox')
        if box is None:  # Archived diagnostics only; production replies carry this compact field.
            box=next(t for t in value.get('diagnostics', {}).get('instances', [])
                     if t['object_type']=='cup')['bbox']
        if (box.get('valid') is not True or box.get('freshness')!='CURRENT'
                or box.get('source_stamp_ns')!=value['source_stamp_ns'] or box.get('frame_id')!=value['frame_id']):
            raise ValueError('current-image cup bounding box required for rim refinement')
        bbox = box['xyxy']
        if len(bbox)!=4 or not all(type(v) in (int,float) and math.isfinite(v) for v in bbox):
            raise ValueError('finite current-image cup bounding box required')
        with np.load(frame_path, allow_pickle=False) as frame:
            rgb, k, camera, q = (frame[n] for n in ('rgb', 'K', 'position', 'quaternion_wxyz'))
        x0, y0, x1, y1 = [int(v) for v in bbox]
        if x1<=x0 or y1<=y0:
            raise ValueError('positive cup bounding box required')
        pad = max(3, math.ceil(.1 * max(x1 - x0, y1 - y0)))
        x0, y0 = max(0, x0 - pad), max(0, y0 - pad)
        x1, y1 = min(rgb.shape[1], x1 + pad), min(rgb.shape[0], y1 + pad)
        hsv = cv2.cvtColor(rgb[y0:y1, x0:x1], cv2.COLOR_RGB2HSV)
        mask = ((hsv[:, :, 1] < 60) & (hsv[:, :, 2] > 160)).astype(np.uint8) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = [c for c in contours if len(c) >= 16 and cv2.contourArea(c) > 100]
        if not contours:
            raise ValueError('visible white cup rim is missing')
        contour = max(contours, key=cv2.contourArea) + np.array([[[x0, y0]]])
        ellipse = cv2.fitEllipse(contour)
        fit = fit_translation(ellipse, k, camera, q, cup['world']['xyz_m'], spec)
        receipt.update(status='accepted', **fit,
            ellipse=dict(center=list(ellipse[0]), diameters=list(ellipse[1]), angle_deg=float(ellipse[2])))
        cup['world']['xyz_m'] = list(fit['world_m'])
        cup['canonical_world']['xyz_m'] = list(fit['world_m'])
        cup['xyz_m'] = list(fit['camera_m'])
    except (ValueError, KeyError, StopIteration, cv2.error) as error:
        receipt.update(status='rejected', reason=str(error))
        value['pair_valid'] = False
        cup['pose_current'] = False
    cup['provenance']['pose_refinement'] = receipt
    return value


def validate_observation(observation, selected_cup, *, input_sha256=None):
    """The source bridge independently checks the declared estimator chain."""
    spec = selected_cup.get('pose_refinement')
    if spec is None or not observation['pair_valid']:
        return
    validate_spec(spec, selected_cup['interaction_profile_sha256'])
    cup = next(t for t in observation['tracks'] if t['object_type'] == 'cup')
    receipt = cup.get('provenance', {}).get('pose_refinement') or {}
    from scene_assets import _sha
    if (receipt.get('method') != METHOD or receipt.get('source_sha256') != source_sha256()
            or receipt.get('spec') != spec or receipt.get('status') != 'accepted'
            or receipt.get('source_stamp_ns') != observation['source_stamp_ns']
            or receipt.get('world_m') != cup['world']['xyz_m']
            or receipt.get('camera_m') != cup['xyz_m']
            or receipt.get('world_m') != cup.get('canonical_world',{}).get('xyz_m')
            or input_sha256 is not None and receipt.get('input_sha256') != _sha(input_sha256)
            or receipt.get('table_z_fixed') is not False or receipt.get('temporal_filter') is not False
            or not 0 <= receipt.get('fit_rms_px', math.inf) <= spec['max_fit_rms_px']
            or not 0 <= receipt.get('position_delta_m', math.inf) < spec['max_position_delta_m']):
        raise ValueError('selected cup rim refinement receipt missing or inconsistent')
