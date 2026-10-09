"""Playground-only same-K rectified RGB display; raw capture is untouched."""
from functools import lru_cache
import json
import math


@lru_cache(maxsize=3)
def display_map(serialized_intrinsics):
    import cv2
    import numpy as np
    from real_observation import inverse_brown_map
    intr = json.loads(serialized_intrinsics)
    if intr['distortion_model'].split('.')[-1] == 'inverse_brown_conrady':
        mapping = inverse_brown_map(serialized_intrinsics).astype(np.float32)
        return mapping[..., 0].copy(), mapping[..., 1].copy()
    K = np.array([[intr['fx'], 0, intr['cx']], [0, intr['fy'], intr['cy']], [0, 0, 1]], dtype=np.float64)
    return cv2.initUndistortRectifyMap(K, np.array(intr['coefficients']), None, K, (640, 480), cv2.CV_32FC1)


def rectified_frame(bridge, name):
    import cv2
    import numpy as np
    if name not in ('base_0', 'left_wrist_0', 'right_wrist_0'):
        raise ValueError('unknown Playground camera')
    status = json.loads(bridge.camera(name, 'status')[0])
    if status.get('ready') is not True or status.get('serial') != bridge.config['cameras'][name]['serial']:
        raise ValueError('Playground camera is unavailable or serial differs')
    intr = status.get('intrinsics', {})
    if (intr.get('width'), intr.get('height')) != (640, 480):
        raise ValueError('Playground rectified display requires 640x480 intrinsics')
    K = [intr.get(k) for k in ('fx', 'fy', 'cx', 'cy')]
    coefficients = intr.get('coefficients')
    if (any(type(v) not in (float, int) or not math.isfinite(v) for v in K) or min(K[:2]) <= 0
            or not isinstance(coefficients, list) or len(coefficients) != 5
            or any(type(v) not in (float, int) or not math.isfinite(v) for v in coefficients)):
        raise ValueError('Playground display requires finite calibrated intrinsics')
    model = str(intr.get('distortion_model', '')).split('.')[-1]
    if model not in ('none', 'brown_conrady', 'inverse_brown_conrady'):
        raise ValueError('unsupported Playground distortion model')
    distorted = intr.get('rectified') is not True and any(abs(v) > 1e-12 for v in coefficients)
    if distorted and model == 'none':
        raise ValueError('nonzero distortion has no declared model')
    body, _ = bridge.camera(name, 'frame.jpg')
    if not isinstance(body, bytes) or len(body) > 8_000_000:
        raise ValueError('bounded camera JPEG required')
    image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (480, 640, 3):
        raise ValueError('Playground camera frame must be 640x480 RGB')
    if not distorted:
        return body
    maps = display_map(json.dumps(intr, sort_keys=True))
    image = cv2.remap(image, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError('rectified display JPEG encoding failed')
    return encoded.tobytes()
