"""Nearest approach between either arm and the camera mast.

This is a *search-time margin*, not a collision judgement: `path_check` already
decides whether a pose or a leg collides. What it cannot express is "this pose
clears the pole by 0.2 mm", which is how the v4 wrist seed came out - safe on
paper, grazing the real hardware. So the seed search asks this module for the
distance from the arms to the mast and refuses anything inside a margin.

What counts as the mast: the parts standing on the centre line above the chassis
plate - the mounting rod, the original mount seat half way up it, the extension
rod and its top assembly, plus the head camera box. Selected geometrically
(|y| <= 6 cm, x >= -1 cm, z >= 2 cm) because the chassis is one mesh file and the
pole has no separate name; the arm bases sit at |y| = 22-28 cm, so they never
match the filter. Configured pole boxes are matched by name.

Distance is read from a trilinear-sampled Euclidean distance field over a grid
around the mast (one build, then ~10 ms per pose), because the honest alternative
- a KD tree over the sampled surface - costs 7.5 s per pose with a 264k-point
tree, and the search evaluates hundreds of poses. The field is conservative: every
sample point's cell is filled, and the value a cell reports is the distance to the
nearest filled cell centre, so the returned distance is reduced by one cell to
stay on the safe side of that discretisation.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt

MAST_HALF_WIDTH_M = 0.06
MAST_MIN_Z_M = 0.02
MAST_MIN_X_M = -0.01
MAST_BOX_NAMES = ('top_camera', 'mast', 'pole', 'stand')
GRID_M = 0.004
PAD_M = 0.25


def mast_triangles(geometry):
    """Full triangles of the mast assembly, as an (n, 3, 3) array."""
    parts = []
    triangles = getattr(geometry, 'static_triangles', [])
    labels = getattr(geometry, 'static_labels', [])
    for block, label in zip(triangles, labels):
        block = np.asarray(block, float).reshape(-1, 3, 3)
        if label in MAST_BOX_NAMES:
            parts.append(block)
            continue
        centre = block.mean(1)
        keep = ((centre[:, 2] >= MAST_MIN_Z_M) & (np.abs(centre[:, 1]) <= MAST_HALF_WIDTH_M)
                & (centre[:, 0] >= MAST_MIN_X_M))
        if keep.any():
            parts.append(block[keep])
    if not parts:
        raise ValueError('no mast geometry found in this configuration')
    return np.concatenate(parts)


def surface_samples(triangles, spacing):
    """Barycentric grid on every triangle; a filled-cell field, not a mesh."""
    points = []
    for triangle in triangles:
        n = max(2, int(max(np.linalg.norm(triangle[1] - triangle[0]),
                           np.linalg.norm(triangle[2] - triangle[0])) / spacing) + 1)
        for a in np.linspace(0, 1, n):
            for b in np.linspace(0, 1 - a, max(2, int((1 - a) * n) + 1)):
                points.append(triangle[0] + a * (triangle[1] - triangle[0]) + b * (triangle[2] - triangle[0]))
    return np.asarray(points)


class MastClearance:
    """Distance from each arm link to the mast surface, in metres."""

    def __init__(self, geometry, *, spacing=GRID_M, pad=PAD_M):
        self.geometry = geometry
        self.spacing, self.pad = float(spacing), float(pad)
        self.triangles = mast_triangles(geometry)
        points = surface_samples(self.triangles, self.spacing * 0.6)
        self.low = points.min(0) - self.pad
        self.shape = np.ceil((points.max(0) + self.pad - self.low) / self.spacing).astype(int)
        occupied = np.zeros(tuple(self.shape), bool)
        index = np.clip(((points - self.low) / self.spacing).astype(int), 0, self.shape - 1)
        occupied[index[:, 0], index[:, 1], index[:, 2]] = True
        # Cell centres, minus one cell so no cell can claim more room than it has.
        self.field = np.maximum(0., distance_transform_edt(~occupied) * self.spacing - self.spacing)
        self.parts = {side: [(name, np.unique(np.asarray(v, float), axis=0))
                             for name, v, _ in meshes]
                      for side, meshes in getattr(geometry, 'side_meshes', {}).items()}

    def _distance(self, points):
        """Trilinear read of the field; anything outside the padded box is `pad` away."""
        relative = (points - self.low) / self.spacing
        top = self.shape - 1
        inside = np.all((relative >= 0) & (relative <= top), axis=1)
        value = np.full(len(points), self.pad)
        if not inside.any():
            return value
        cell = np.clip(np.floor(relative[inside]).astype(int), 0, top - 1)
        weight = np.clip(relative[inside] - cell, 0, 1)
        out = np.zeros(len(weight))
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    corner = weight.copy()
                    for axis, shift in enumerate((dx, dy, dz)):
                        corner[:, axis] = weight[:, axis] if shift else 1 - weight[:, axis]
                    out += np.prod(corner, axis=1) * self.field[cell[:, 0] + dx, cell[:, 1] + dy, cell[:, 2] + dz]
        value[inside] = out
        return value

    def clearance(self, geometry, q, *, want_nearest=False, sides=('left', 'right')):
        frames = geometry.frames(q)
        best, nearest = float('inf'), None
        for side in sides:
            for name, local in self.parts[side]:
                T = frames[side][name]
                points = local @ T[:3, :3].T + T[:3, 3]
                smallest = float(self._distance(points).min())
                if smallest < best:
                    best, nearest = smallest, (side, name)
        return (best, nearest) if want_nearest else best

    def survive(self, geometry, q, margin):
        """True when both arms keep `margin` metres of daylight to the mast."""
        if not margin:
            return True
        return self.clearance(geometry, q) >= margin
