"""Split calibration link6 into native palm and prismatic finger meshes.

The native finger axes are +Y/-Y, joint origins have identity rotation, and each
finger joint position is half the physical metal gap in metres. No SDK mapping
is involved. Input vertices already include the calibration visual origin/scale.
"""
import numpy as np


def split_hand_meshes(hand, joint_origins):
    """Return {0: palm, +1: positive finger, -1: negative finger}, (v, f) each.

    Palm vertices remain in link6 coordinates. Finger vertices are transformed
    into their native child-link coordinates at zero physical metal gap.
    """
    vertices = np.asarray(hand['vertices'], dtype=float)
    faces = np.asarray(hand['faces'])
    signs = np.asarray(hand['signs'])
    gap = float(hand['nominal_gap_m'])
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all()
            or signs.shape != (len(vertices),) or set(np.unique(signs)) != {-1, 0, 1}
            or faces.ndim != 2 or faces.shape[1] != 3 or not np.issubdtype(faces.dtype, np.integer)
            or faces.size == 0 or faces.min() < 0 or faces.max() >= len(vertices)
            or not .04 <= gap <= .08):
        raise ValueError('invalid calibration hand template')
    face_signs = signs[faces]
    if np.any(face_signs != face_signs[:, :1]):
        raise ValueError('hand triangles cross moving components')
    result = {}
    for sign in (0, 1, -1):
        selected = faces[face_signs[:, 0] == sign]
        if not len(selected):
            raise ValueError('missing palm or finger triangles')
        indices, inverse = np.unique(selected, return_inverse=True)
        local = vertices[indices].copy()
        if sign:
            origin = np.asarray(joint_origins[sign], dtype=float)
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError('native finger joint origin must be finite xyz')
            local -= origin
            local[:, 1] -= sign * gap / 2
        result[sign] = local, inverse.reshape(-1, 3)
    return result


if __name__ == '__main__':
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 0, 1],
                  [0, .03, 0], [1, .03, 0], [0, .03, 1],
                  [0, -.03, 0], [1, -.03, 0], [0, -.03, 1]])
    template = dict(vertices=v, faces=np.arange(9).reshape(-1, 3),
                    signs=np.repeat([0, 1, -1], 3), nominal_gap_m=.06)
    origins = {1: np.array([.08397, .02721, -.0062702]),
               -1: np.array([.08397, -.023638, -.0062703])}
    meshes = split_hand_meshes(template, origins)
    for gap in (0., .03, .083):
        for sign, (local, faces) in meshes.items():
            world = local.copy()
            if sign:
                world += origins[sign]
                world[:, 1] += sign * gap / 2
            expected = v[template['signs'] == sign].copy()
            expected[:, 1] += sign * (gap - .06) / 2
            assert np.allclose(world[faces].reshape(-1, 3), expected, atol=1e-15)
    broken = dict(template, faces=np.array([[0, 3, 6]]))
    try:
        split_hand_meshes(broken, origins)
    except ValueError:
        pass
    else:
        raise AssertionError('cross-component face accepted')
    print('calibrated gripper: native prismatic FK equals calibrated gap at 0/30/83 mm PASS')
