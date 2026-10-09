"""Deterministic nominal joint previews for isolated Isaac runs, never hardware.

Only reported joints enter this planner. Hidden encoder offsets belong to the
simulator executor. Collision clearance is not certified by this helper.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

def vector(value, n, label):
    a = np.asarray(value)
    if a.shape != (n,) or a.dtype.kind not in 'iuf' or not np.isfinite(a).all():
        raise ValueError(f'{label}: expected {n} finite numbers')
    return a.astype(float)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()


JOINT_NAMES = [*(f'left_joint{i}' for i in range(1, 7)), *(f'right_joint1{i}' for i in range(1, 7))]
IDENTITY_KEYS = ('scene_sha256', 'asset_sha256', 'nominal_camera_sha256')
SPEED = math.radians(8)
MARGIN = math.radians(5)


def _identity(value):
    if set(value) != set(IDENTITY_KEYS):
        raise ValueError('scene, asset and nominal camera SHA256 identities required')
    for key, item in value.items():
        if not isinstance(item, str) or len(item) != 64 or any(c not in '0123456789abcdef' for c in item):
            raise ValueError(f'invalid {key}')
    return dict(value)


def _limits(urdf):
    joints = {row.get('name'): row for row in ET.parse(urdf).getroot().findall('joint')}
    limits = []
    for name in JOINT_NAMES:
        joint = joints.get(name)
        if joint is None or joint.get('type') != 'revolute' or joint.find('limit') is None:
            raise ValueError(f'bounded revolute joint required: {name}')
        limit = joint.find('limit')
        limits.append([float(limit.get('lower')), float(limit.get('upper'))])
    result = np.asarray(limits)
    if not np.isfinite(result).all() or np.any(result[:, 1] - result[:, 0] <= 2 * MARGIN):
        raise ValueError('invalid joint limits or insufficient margin')
    return result


def preview_plan(urdf, identity, q_current_rad, q_target_rad=None, *, kind='target', seed=17):
    """Preview a full 12-joint target, moving left then right, with 5° margins.

    Existing hard-stop positions may be held; moved joints must end inside the
    conservative interval. ``home`` means nominal zero and may be
    rejected on AC-One J2/J3 lower stops; it never silently means 5 degrees.
    """
    identity = _identity(identity)
    if type(seed) is not int or not 0 <= seed <= 2**32 - 1:
        raise ValueError('seed must be a uint32 integer')
    if kind not in ('target', 'home') or (kind == 'home' and q_target_rad is not None):
        raise ValueError('expected target or home without a target override')
    current = vector(q_current_rad, 12, 'reported current joints')
    target = np.zeros(12) if kind == 'home' else vector(q_target_rad, 12, 'reported target joints')
    limits = _limits(urdf)
    # The planner cannot know encoder bias; the world-side executor must check
    # physical limits independently before sending its first servo target.
    if np.any(current < limits[:, 0] - MARGIN - 1e-9) or np.any(current > limits[:, 1] + MARGIN + 1e-9):
        raise ValueError('reported current joints exceed nominal limits plus 5 degree calibration allowance')
    moved = np.ones(12, dtype=bool) if kind == 'home' else abs(target - current) > 1e-12
    outside = (target < limits[:, 0] + MARGIN) | (target > limits[:, 1] - MARGIN)
    if np.any(moved & outside):
        raise ValueError(f'{kind} target violates 5 degree nominal limit margin; no motion authorized')
    urdf_hash = hashlib.sha256(Path(urdf).read_bytes()).hexdigest()
    trajectory = [{'time_s': 0., 'q_reported_rad': current.tolist(), 'active_arm': None}]
    q, time_s = current.copy(), 0.
    for side, selection in (('left', slice(0, 6)), ('right', slice(6, 12))):
        delta = target[selection] - q[selection]
        distance = float(np.max(abs(delta)))
        if distance <= 1e-12:
            continue
        # Cubic smoothstep's maximum derivative is 1.5; linear interpolation
        # between these samples also stays below the continuous speed bound.
        duration = max(.1, 1.5 * distance / SPEED)
        count = math.ceil(duration / .1)
        start = q.copy()
        for index in range(1, count + 1):
            u = index / count
            q[selection] = start[selection] + delta * (3 * u * u - 2 * u * u * u)
            trajectory.append({'time_s': time_s + duration * u,
                               'q_reported_rad': q.tolist(), 'active_arm': side})
        time_s += duration
    plan = {'schema': 'joint-head-isaac-plan-v1', 'kind': kind, 'seed': seed,
            'joint_names': JOINT_NAMES.copy(), 'identity': identity, 'urdf_sha256': urdf_hash,
            'start_q_reported_rad': current.tolist(), 'target_q_reported_rad': target.tolist(),
            'start_fingerprint': digest({'identity': identity, 'urdf_sha256': urdf_hash,
                                         'q_reported_rad': current.tolist()}),
            'duration_s': time_s, 'trajectory': trajectory,
            'max_speed_rad_s': SPEED, 'max_waypoint_interval_s': .1, 'limit_margin_rad': MARGIN,
            'uncalibrated_limit_allowance_rad': MARGIN,
            'current_outside_nominal_limits': bool(np.any((current < limits[:, 0]) | (current > limits[:, 1]))),
            'physical_limit_check_required': True,
            'simulation_only': True, 'hardware_executable': False, 'collision_certified': False}
    plan['hash'] = digest(plan)
    return plan


def validate_plan(plan, urdf, identity):
    """Rebuild canonical trajectory, rejecting changed identities and waypoints."""
    if not isinstance(plan, dict) or not isinstance(plan.get('hash'), str):
        raise ValueError('missing plan seal')
    if digest({k: v for k, v in plan.items() if k != 'hash'}) != plan['hash']:
        raise ValueError('plan content changed')
    expected = preview_plan(urdf, identity, plan['start_q_reported_rad'],
                            None if plan['kind'] == 'home' else plan['target_q_reported_rad'],
                            kind=plan['kind'], seed=plan['seed'])
    if expected != plan:
        raise ValueError('plan is not canonical or source identity changed')
    return plan


def authorize_plan(plan, urdf, identity, q_current_rad, confirmed_hash, consumed):
    """Consume exact confirmation once; call under the executor's operation lock.

    ``consumed`` is an in-memory set owned by one isolated simulation session;
    it is not durable replay protection and must never guard hardware actions.
    """
    validate_plan(plan, urdf, identity)
    if confirmed_hash != plan['hash'] or confirmed_hash in consumed:
        raise ValueError('unconfirmed or already consumed plan')
    current = vector(q_current_rad, 12, 'reported current joints')
    if np.max(abs(current - plan['start_q_reported_rad'])) > .001:
        raise ValueError('start state drifted; preview again')
    consumed.add(confirmed_hash)
    return deepcopy(plan)
