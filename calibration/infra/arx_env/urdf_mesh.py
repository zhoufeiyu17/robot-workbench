"""Small offline URDF/STL projection input loader. Never imports a robot SDK."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    x, y, z = np.asarray(rpy, float)
    cx, cy, cz, sx, sy, sz = np.cos(x), np.cos(y), np.cos(z), np.sin(x), np.sin(y), np.sin(z)
    T = np.eye(4)
    T[:3, :3] = [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                 [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]]
    T[:3, 3] = xyz
    if not np.isfinite(T).all():
        raise ValueError("nonfinite transform")
    return T


def origin(element):
    if element is None:
        return np.eye(4)
    return transform([float(x) for x in element.get("xyz", "0 0 0").split()],
                     [float(x) for x in element.get("rpy", "0 0 0").split()])


def relative_file(root, value):
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("dataset paths must stay relative to their root")
    path = root / relative
    if path.is_symlink() or path.resolve(strict=True) != path or not path.is_file():
        raise ValueError("dataset source must be a canonical regular file")
    return path
