"""Synthetic images only: same-K display, identity checks and raw route isolation."""
from copy import deepcopy
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from playground_display import display_map, rectified_frame
from real_observation import inverse_brown_rectified_pixels


def main():
    intr = dict(width=640, height=480, fx=389.2, fy=389.1, cx=319.4, cy=243.7,
                distortion_model='distortion.inverse_brown_conrady',
                coefficients=[-.025, .003, .0002, -.0003, 0.], rectified=False)
    original = deepcopy(intr)
    status = dict(ready=True, serial='synthetic-head', intrinsics=intr)
    yy, xx = np.indices((480, 640))
    pixels = np.stack((xx % 256, yy % 256, (xx+yy) % 256), axis=-1).astype(np.uint8)
    _, jpeg = cv2.imencode('.jpg', pixels)
    raw = jpeg.tobytes()
    bridge = SimpleNamespace(config={'cameras': {'base_0': {'serial': 'synthetic-head'}}})
    bridge.camera = Mock(side_effect=lambda name, resource: (json.dumps(status).encode(), 'application/json')
                         if resource == 'status' else (raw, 'image/jpeg'))
    result = rectified_frame(bridge, 'base_0')
    assert cv2.imdecode(np.frombuffer(result, np.uint8), cv2.IMREAD_COLOR).shape == (480, 640, 3)
    assert result != raw and intr == original, 'display rectification must not mutate calibration/raw'
    maps = display_map(json.dumps(intr, sort_keys=True))
    assert maps is display_map(json.dumps(intr, sort_keys=True)), 'mapping cached by exact intrinsics'
    sample = np.array([[100, 100], [320, 240], [500, 350]])
    raw_pixels = np.stack([maps[0][sample[:, 1], sample[:, 0]], maps[1][sample[:, 1], sample[:, 0]]], axis=-1)
    assert np.max(np.abs(inverse_brown_rectified_pixels(raw_pixels, intr)-sample)) < .001
    rectified_frame(bridge, 'base_0')
    assert display_map.cache_info().hits >= 2
    intr['rectified'] = True
    assert rectified_frame(bridge, 'base_0') == raw, 'already rectified image cannot be rectified twice'
    intr.update(original)
    for mutation in (lambda: status.update(serial='wrong-camera'), lambda: intr.update(width=1280),
                     lambda: intr.update(fx=float('nan')), lambda: intr.update(distortion_model='unsupported')):
        status['serial'] = 'synthetic-head'; intr.clear(); intr.update(original)
        mutation(); bridge.camera.reset_mock()
        try:
            rectified_frame(bridge, 'base_0')
            raise AssertionError('invalid camera metadata accepted')
        except ValueError:
            pass
        assert len(bridge.camera.call_args_list) == 1, 'reject identity before image fetch'
    status['serial'] = 'synthetic-head'; intr.clear(); intr.update(original)
    import app
    from test_app import serving
    with patch.object(app, 'REAL_BRIDGE', bridge), serving(app.WorkbenchState()) as request:
        rendered = request('GET', '/api/playground/camera/base_0/frame.jpg')
        assert rendered == result
        assert request('GET', '/api/real/camera/base_0/frame.jpg') == raw, 'old raw preview route must not change'
        request('GET', '/api/playground/camera/base_0/snapshot.zip', expected=503)
        request('GET', '/api/playground/camera/unknown/frame.jpg', expected=503)
        request('GET', '/api/playground/camera/base_0/frame.jpg', headers={'Origin': 'http://evil.example'}, expected=403)
    print('PASS same-K cached inverse-Brown display, RGB size/identity checks, no double rectification and raw API isolation')


if __name__ == '__main__':
    main()
