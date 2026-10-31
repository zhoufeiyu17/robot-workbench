"""Offline gravity features and empirical output models; never sends commands."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from arx_env.joint_head_isaac_fit import Model


def vector(value, size, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{name} requires {size} finite values')
    return value


class GravityFeatures:
    """Link-frame mass/first-moment features on already-calibrated geometry."""
    def __init__(self, urdf, joint_names, gravity=(0., 0., -9.807)):
        self.model = Model(urdf, joint_names)
        self.gravity = vector(gravity, 3, 'gravity')
        self.joints = {r[0]: r for r in self.model.joints}
        parents = {r[2]: r[1] for r in self.model.joints}
        self.upstream = {}
        for link in self.model.links:
            ancestors = set(); current = link
            while current in parents:
                ancestors.add(current); current = parents[current]
            self.upstream[link] = [i for i, name in enumerate(self.model.names)
                                   if self.joints[name][2] in ancestors]
        self.links, parameters = [], []
        for row in ET.parse(Path(urdf)).getroot().findall('link'):
            inertia = row.find('inertial')
            if inertia is None:
                continue
            mass = float(inertia.find('mass').get('value'))
            origin = inertia.find('origin')
            com = vector([float(x) for x in ('0 0 0' if origin is None else origin.get('xyz', '0 0 0')).split()], 3, 'COM')
            if not np.isfinite(mass) or mass < 0:
                raise ValueError('nonnegative finite mass required')
            self.links.append(row.get('name')); parameters.extend([mass, *(mass * com)])
        self.nominal_parameters = np.asarray(parameters)

    def _link_matrix(self, frames, link):
        if link not in self.model.links:
            raise ValueError('payload attachment link is not in the frozen geometry')
        result = np.zeros((len(self.model.names), 4))
        transform = frames[link]
        for i in self.upstream[link]:
            _, parent, _, origin, axis, _ = self.joints[self.model.names[i]]
            joint = frames[parent] @ origin
            axis_world = joint[:3, :3] @ axis
            result[i, 0] = -axis_world @ np.cross(transform[:3, 3] - joint[:3, 3], self.gravity)
            for k in range(3):
                result[i, k + 1] = -axis_world @ np.cross(transform[:3, k], self.gravity)
        return result

    def matrix(self, q):
        frames = self.model.frames(q)
        return np.concatenate([self._link_matrix(frames, link) for link in self.links], axis=1)

    def nominal(self, q):
        return self.matrix(q) @ self.nominal_parameters

    def payload(self, q, *, attachment_link, mass_kg, com_m, output_units_per_nm):
        """Additional payload only; conversion must come from an explicit contract.

        COM is expressed in attachment_link, not camera/world coordinates. This
        geometric calculation alone does not qualify its scale on hardware.
        """
        if isinstance(mass_kg, bool) or not np.isfinite(mass_kg) or mass_kg < 0:
            raise ValueError('nonnegative finite payload mass required')
        com = vector(com_m, 3, 'payload COM')
        scale = vector(output_units_per_nm, len(self.model.names), 'per-axis output conversion')
        if np.any(scale <= 0):
            raise ValueError('positive per-axis output conversion required')
        basis = self._link_matrix(self.model.frames(q), attachment_link)
        return (basis @ np.r_[mass_kg, mass_kg * com]) * scale


def fit_residual(features, residual_output, approach_direction, *, rcond):
    """Fit each axis in its own units, separating a direction-dependent nuisance.

    Inputs contain only the fixed-attachment baseline, never unknown payloads.
    rcond is an explicit numerical regularization choice, not a control limit.
    Rank/singular values must be reviewed; a returned fit is not a deployable model.
    """
    features = np.asarray(features, dtype=float)
    target = np.asarray(residual_output, dtype=float)
    directions = np.asarray(approach_direction, dtype=float)
    if (features.ndim != 3 or features.shape[:2] != target.shape or target.shape != directions.shape
            or features.shape[0] < 3 or features.shape[2] == 0
            or not all(np.isfinite(x).all() for x in (features, target, directions))
            or not np.isin(directions, [-1., 0., 1.]).all()
            or not np.isfinite(rcond) or not 0 < rcond < 1):
        raise ValueError('finite aligned samples/axes/features, directions and 0<rcond<1 required')
    axes = []
    for i in range(target.shape[1]):
        x = np.column_stack((features[:, i], np.ones(len(target)), directions[:, i]))
        scale = np.linalg.norm(x, axis=0); scale[scale == 0] = 1
        normalized = x / scale
        coefficients, _, rank, singular = np.linalg.lstsq(normalized, target[:, i], rcond=rcond)
        coefficients /= scale
        axes.append(dict(coefficients=coefficients.tolist(),rank=int(rank),columns=x.shape[1],
                         singular_values=singular.tolist(),training_max_abs_error=float(np.max(abs(x @ coefficients - target[:, i])))))
    return dict(schema='offline_gravity_residual.v1',axes=axes,feature_count=features.shape[2],
                rcond=float(rcond),hardware_qualified=False,payload_calibrated=False)


def predict_residual(model, features, approach_direction, *, output_limits):
    features = np.asarray(features, dtype=float)
    directions = np.asarray(approach_direction, dtype=float)
    n = len(model['axes']); limits = vector(output_limits, n, 'output limits')
    if (features.ndim != 3 or features.shape[1:] != (n, model['feature_count'])
            or directions.shape != features.shape[:2] or not np.isfinite(features).all()
            or not np.isin(directions, [-1., 0., 1.]).all() or np.any(limits <= 0)):
        raise ValueError('valid aligned prediction inputs and positive explicit limits required')
    output = np.column_stack([np.column_stack((features[:, i], np.ones(len(features)), directions[:, i]))
                              @ vector(axis['coefficients'], model['feature_count'] + 2, 'coefficients')
                              for i, axis in enumerate(model['axes'])])
    if not np.isfinite(output).all() or np.any(abs(output) > limits):
        raise ValueError('prediction exceeds explicit output envelope; no silent clipping')
    return output
