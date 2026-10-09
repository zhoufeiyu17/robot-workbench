#!/usr/bin/env python3
"""Bounded, camera-only sticker preview. No robot SDK, motion, or calibration fit.

Raw distorted images are for placement checks only; no synchronized joints exist.
"""
import argparse
import json
import socket
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


def marker_check(image, expected):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco,'ArucoDetector'):
        corners,ids,_=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters()).detectMarkers(image)
    else:
        corners,ids,_=cv2.aruco.detectMarkers(image,dictionary,parameters=cv2.aruco.DetectorParameters_create())
    rows = []
    annotated = image.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        for marker_id, xy in zip(ids.ravel(), corners):
            xy = xy.reshape(4, 2)
            side = float(np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1).min())
            margin = float(np.minimum(xy, np.array(image.shape[1::-1]) - 1 - xy).min())
            rows.append(dict(id=int(marker_id), minimum_side_px=side,
                             border_margin_px=margin, expected=int(marker_id) in expected,
                             placement_size_pass=side >= 24 and margin >= 4))
    return annotated, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--left-id', type=int, default=24)
    parser.add_argument('--right-id', type=int, default=30)
    parser.add_argument('--port', type=int, default=8114)
    parser.add_argument('--duration-s', type=int, default=1800)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not (0 <= args.left_id < 50 and 0 <= args.right_id < 50 and args.left_id != args.right_id):
        parser.error('two distinct DICT_4X4_50 IDs required')
    if not 0 <= args.duration_s <= 3600 or not 1024 <= args.port <= 65535:
        parser.error('duration 0 (persistent) or 1..3600 seconds and unprivileged port required')
    args.output.mkdir(parents=False, exist_ok=False)
    lock = threading.Condition()
    latest = {'status': {'ready': False}, 'jpeg': None, 'raw_jpeg': None}

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def log_message(self, *unused):
            pass

        def do_GET(self):
            if self.path.split('?')[0] == '/stream.mjpg':
                self.connection.settimeout(3.)
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                previous = None
                try:
                    while True:
                        with lock:
                            lock.wait_for(lambda: latest['status'].get('frame_number') != previous, timeout=2)
                            status, jpeg = dict(latest['status']), latest['jpeg']
                            if jpeg is None or status.get('frame_number') == previous:
                                return
                            previous = status['frame_number']
                            status['source_age_s'] = max(0, (time.time_ns()-int(status['host_receive_ns']))/1e9)
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()
                                         +b'\r\nX-Camera-Metadata: '+json.dumps(status, separators=(',', ':')).encode()
                                         +b'\r\n\r\n'+jpeg+b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass
                return
            status = None
            with lock:
                if self.path == '/status':
                    status=dict(latest['status'])
                    if status.get('host_receive_ns'):status['source_age_s']=max(0,(time.time_ns()-int(status['host_receive_ns']))/1e9)
                    payload, mime = json.dumps(status).encode(), 'application/json'
                elif self.path.split('?')[0] == '/frame.jpg' and latest['jpeg'] is not None:
                    payload, mime = latest['jpeg'], 'image/jpeg'
                    status = dict(latest['status'])
                    status['source_age_s'] = max(0, (time.time_ns()-int(status['host_receive_ns']))/1e9)
                elif self.path.split('?')[0] == '/raw-frame.jpg' and latest['raw_jpeg'] is not None:
                    payload, mime = latest['raw_jpeg'], 'image/jpeg'
                    status = dict(latest['status'], raw=True, annotated=False, image_domain='raw')
                    status['source_age_s'] = max(0, (time.time_ns()-int(status['host_receive_ns']))/1e9)
                elif self.path == '/':
                    payload = ('''<!doctype html><meta charset="utf-8"><title>Head camera placement</title>
<style>body{background:#171a20;color:#eee;font:18px sans-serif;margin:24px}img{max-width:100%}pre{white-space:pre-wrap}</style>
<h2>Left ID ''' + str(args.left_id) + ' / Right ID ' + str(args.right_id) + '''</h2>
<p>Camera-only raw preview. No robot control. Not synchronized calibration data.</p>
<img id="v" src="/stream.mjpg"><pre id="s"></pre><script>async function tick(){try{
let r=await fetch('/status',{cache:'no-store'});let x=await r.json();
document.getElementById('s').textContent=JSON.stringify(x,null,2);

}catch(e){document.getElementById('s').textContent='Disconnected: '+e;}}
tick();setInterval(tick,1000);</script>''').encode()
                    mime = 'text/html; charset=utf-8'
                else:
                    self.send_error(404)
                    return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            if mime == 'image/jpeg' and status is not None:
                self.send_header('X-Camera-Metadata', json.dumps(status, separators=(',', ':')))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    running = False
    try:
        config = rs.config()
        config.enable_device(args.serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.yuyv, 30)
        profile = pipeline.start(config)
        running = True
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        metadata = dict(serial=args.serial, width=intr.width, height=intr.height,
                        fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy,
                        distortion_model=str(intr.model), coefficients=list(intr.coeffs),
                        rectified=False, left_id=args.left_id, right_id=args.right_id,
                        dictionary='DICT_4X4_50', black_size_m=0.015,
                        joint_feedback_available=False, calibration_sample=False,
                        motion_commanded=False)
        (args.output / 'camera.json').write_text(json.dumps(metadata, indent=2))
        deadline = time.monotonic() + args.duration_s if args.duration_s else float("inf")
        counter = 0
        archive_at = time.monotonic()
        rate_at, rate_count, delivered_fps = time.monotonic(), 0, None
        while time.monotonic() < deadline:
            frame = pipeline.wait_for_frames(3000).get_color_frame()
            if not frame:
                raise RuntimeError('missing color frame')
            stamp = time.time_ns()
            counter += 1
            if counter < 20:
                continue
            data = np.asanyarray(frame.get_data()).view(np.uint8).reshape(480, 640, 2)
            raw = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_YUY2)
            annotated, rows = marker_check(raw, [args.left_id, args.right_id])
            rate_count += 1
            elapsed = time.monotonic()-rate_at
            if elapsed >= 1:
                delivered_fps = rate_count/elapsed
                rate_at, rate_count = time.monotonic(), 0
            status = dict(ready=True, host_receive_ns=str(stamp), capture_fps=30, delivered_fps=delivered_fps,
                          serial=args.serial, intrinsics=metadata, raw=False, annotated=True, image_domain='raw',
                          frame_number=frame.get_frame_number(),
                          device_timestamp_ms=frame.get_timestamp(),
                          timestamp_domain=str(frame.get_frame_timestamp_domain()),
                          left_id=args.left_id, right_id=args.right_id, detections=rows,
                          calibration_sample=False, motion_commanded=False,
                          seconds_remaining=max(0, round(deadline-time.monotonic())) if args.duration_s else None)
            ok, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            payload = jpeg.tobytes()
            ok, raw_jpeg = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError('raw JPEG encoding failed')
            with lock:
                latest.update(status=status, jpeg=payload, raw_jpeg=raw_jpeg.tobytes())
                lock.notify_all()
            # Throttle compatibility snapshots; live viewers read memory, never files.
            if time.monotonic() >= archive_at:
                archive_at = time.monotonic()+1
                for name, data in [('latest.png', cv2.imencode('.png', raw)[1].tobytes()),
                                   ('annotated.jpg', payload),
                                   ('status.json', json.dumps(status, indent=2).encode())]:
                    temporary = args.output / (name + '.tmp')
                    temporary.write_bytes(data)
                    temporary.replace(args.output / name)

    finally:
        if running:
            pipeline.stop()
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
