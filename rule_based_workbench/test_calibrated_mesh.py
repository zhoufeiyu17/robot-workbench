"""Run with calibration infra on PYTHONPATH and a frozen --snapshot directory."""
import json
import sys
from pathlib import Path

import numpy as np
from calibrated_mesh import CalibratedMesh, load_snapshot
from arx_env.joint_head_workbench_geometry import Geometry


def check(snapshot):
    model = CalibratedMesh(snapshot)
    q = np.array([0, 1, 1, 0, 0, 0] * 2, float)
    meshes, state = model.scene(q, [80, 60])
    np.testing.assert_array_equal(state['q_model_rad'], q + model.offsets)
    np.testing.assert_array_equal(q, [0, 1, 1, 0, 0, 0] * 2)
    assert state['acceptance']['status'] == 'user_accepted_provisional'
    assert state['physical_geometry_verified'] is False
    assert state['markers_rendered'] is False
    original = Geometry(load_snapshot(snapshot)[0])
    for side in ('left', 'right'):
        for (name, vertices, faces), (base_name, base_vertices, base_faces) in zip(
                model.geometry.side_meshes[side], original.side_meshes[side]):
            assert name == base_name and 'marker' not in name and 'sticker' not in name
            np.testing.assert_array_equal(vertices, base_vertices)
            np.testing.assert_array_equal(faces, base_faces)
    for a, b in zip(model.geometry.static_meshes, original.static_meshes):
        assert a[0] == b[0]
        np.testing.assert_array_equal(a[1], b[1])
    trajectory = np.stack([q, q + .01, q - .02])
    np.testing.assert_allclose(model.model_to_reported(model.reported_to_model(trajectory)), trajectory, atol=1e-15)
    np.testing.assert_allclose(model.reported_to_model(model.model_to_reported(trajectory)), trajectory, atol=1e-15)
    assert model.calibration()['markers_rendered'] is False
    frames = model.geometry.frames(q + model.offsets)
    for side in ('left', 'right'):
        np.testing.assert_allclose(state['T_world_optical'][side],
                                   frames[side]['link6'] @ model.wrist_cameras[side])
    poses = model.camera_poses(q)
    moved_q = q.copy(); moved_q[0] += .1
    moved = model.camera_poses(moved_q)
    assert not np.allclose(poses['left'], moved['left'])
    np.testing.assert_array_equal(poses['head'], moved['head'])
    np.testing.assert_array_equal(poses['right'], moved['right'])
    closed, _ = model.scene(q, [0, 0])
    opened, _ = model.scene(q, [80, 60])
    # Only the two finger meshes move when gaps change; repeat calls do not drift.
    assert {n for (n, a), (_, b) in zip(closed, opened) if not np.array_equal(a, b)} == {'left_link6', 'right_link6'}
    assert all(np.array_equal(a, b) for (_, a), (_, b) in zip(meshes, opened))
    for bad in ([0] * 14, [float('nan')] * 12):
        try:
            model.scene(bad, [80, 80])
        except ValueError:
            pass
        else:
            raise AssertionError('invalid reported joints accepted')
    print(json.dumps({'passed': True, 'parts': len(meshes),
                      'triangles': sum(len(v)//3 for _, v in meshes),
                      'checked': ['offset once', 'FK camera chain', 'independent jaw gaps',
                                  'no mutation/drift', 'provisional status', 'invalid input rejection',
                                  'original mesh vertices unchanged', 'no stickers rendered',
                                  'trajectory roundtrip', 'independent wrist cameras']}))


if __name__ == '__main__':
    check(Path(sys.argv[1]))
