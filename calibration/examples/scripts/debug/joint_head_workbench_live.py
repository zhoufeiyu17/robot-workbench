#!/usr/bin/env python3
"""Authenticated loopback persistent Isaac RTX kinematic workbench; no robot IO.

HTTP threads validate/enqueue only. USD and rendering stay on the main thread.
Frames combine orbit overview and the calibrated head optical camera.
"""
import argparse
from collections import deque
import copy
import hashlib
import hmac
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import shutil
import socket
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

import numpy as np
from arx_env.joint_head_workbench_geometry import jaw_gaps


class Session:
    STANDARD_MOTION_SEED = 'x5-standard-head-motion-seed-v1'
    def __init__(self, geometry, output, config):
        self.geometry, self.output = geometry, Path(output)
        self.display_only=config.get('display_only') is True
        self.active_display_view='target' if self.display_only else None
        self.asset_identity = {key: config[key] for key in ('asset_geometry_id', 'asset_seal_sha256') if config.get(key)}
        self.output.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.frame_ready = threading.Condition(self.lock)
        self.mirror = None
        self.mirror_sequences = {}
        self.render_times = {view: deque(maxlen=90) for view in ('actual', 'preview')}
        self.render_ms = {}
        self.render_stages = {}
        self.actual_render_started = None
        self.jaw_mm = jaw_gaps([config.get('gap_m',{}).get(side,.083)*1000 for side in ('left','right')])
        self.q = self.vector(config.get('initial_q', [0.] * 12))
        self.actual_q = self.q.copy()
        self.actual_gaps = self.jaw_mm.copy()
        self.speed_limits = np.minimum(geometry.velocity_limits, config.get('simulation_speed_limit_rad_s', 1.))
        self.status, self.reason = 'STARTING', None
        self.epoch = 0
        self.stopped = threading.Event()
        self.pending = None
        self.preview = None
        self.last_preview_pose = None
        self.preview_revision = -1
        self.renderer_instance_id = uuid.uuid4().hex
        self.frame_metadata = {}
        self.overview_revision = 0
        self.overview_requested_revision = 0
        self.last_preview_received = float('-inf')
        self.last_overlay_started = float('-inf')
        self.last_wrist_overlay_started = float('-inf')
        self.overview_view = None
        self.frame_sequence = 0
        self.frame_timestamp = {}
        self.actual_head = None
        self.actual_cameras = {}
        self.wrist_contract = None
        self.pending_wrist_contract = None
        self.simulation_truth_reference = None
        self.pending_wrist_config = None
        if config.get('model_family') == 'ac_one' and config.get('wrist_simulation_cameras'):
            from joint_head_wrist_fixture_bringup import nominal_fixture_contract
            self.pending_wrist_config = validate_wrist_contract(nominal_fixture_contract(config))
        self.wrist_calibrations = {}
        self.wrist_overlay_busy = False
        self.wrist_overlay_revision = 0
        self.wrist_overlay_state = {}
        self.mesh_rendering_enabled = config.get('mesh_rendering_enabled',False) is True
        self.calibration = None
        self.calibration_revision = 0
        self.calibration_busy = False
        self.calibration_state = {'id': None, 'ready': False, 'error': None}
        self.jpeg = {}
        self.updated = time.time()
        self.client_seen = time.monotonic()
        self.active = None
        self.validation = False
        self.capture_id = None
        self.dataset = None
        self.batch_dir = None
        self.rows = []
        self.planned_frames = []
        self.progress = {'index': 0, 'total': 0, 'samples': 0, 'total_samples': 0}
        self.urdf = config.get('urdf')
        self.camera = {**geometry.k, 'coefficients': [0.]*5, 'rectified': True,
                       'distortion_model': 'distortion.inverse_brown_conrady'}

    def vector(self, value):
        q = np.asarray(value, float)
        if q.shape != (12,) or not np.isfinite(q).all():
            raise ValueError('12 finite joint values required')
        if np.any(q < self.geometry.limits[:, 0]) or np.any(q > self.geometry.limits[:, 1]):
            raise ValueError('joint limits exceeded')
        return q

    def state(self):
        with self.lock:
            self.client_seen = time.monotonic()
            self.expire_mirror()
            return {'mode': 'simulation', 'mesh_rendering_enabled':self.mesh_rendering_enabled, 'active_display_view':self.active_display_view, 'state': self.status, 'status': self.status,
                    'renderer_instance_id':self.renderer_instance_id,'overview_revision':self.overview_revision,
                    **self.asset_identity,
                    'jaw_gap_mm':self.jaw_mm.tolist(), 'q': self.q.tolist(), 'qpos12': self.q.tolist(), 'timestamp_s': self.updated,
                    'age_s': max(0, time.time()-self.updated), 'preview_revision': self.preview_revision,
                    'frame_sequence': self.frame_sequence, 'frame_timestamp_s': dict(self.frame_timestamp),
                    'progress': dict(self.progress),
                    'mirror': None if self.mirror is None else {**self.mirror, 'age_s': time.monotonic()-self.mirror['received_at']},
                    'render': {view: {'fps': (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0., 'last_duration_ms': self.render_ms.get(view), 'stages_ms': self.render_stages.get(view, {}), 'target_hz': 30} for view,times in self.render_times.items()},
                    'capture_id': self.capture_id, 'dataset_ready': self.dataset is not None,
                    'calibration': dict(self.calibration_state),
                    'wrist_calibration': copy.deepcopy(self.wrist_overlay_state),
                    'wrist_snapshot_sha256': self.wrist_contract['head_snapshot']['snapshot_sha256'] if self.wrist_contract else None,
                    'wrist_contract_sha256': contract_sha256(self.wrist_contract) if self.wrist_contract else None,
                    'wrist_config_pending': self.pending_wrist_config is not None,
                    'reason': self.reason, 'feedback_kind': 'real joint feedback mirrored in DSW' if self.mirror else 'DSW RTX commanded kinematic state; not motor feedback',
                    'capabilities': {'jaw_gap_mm':True, 'preview': True, 'execute': not self.display_only and self.mirror is None, 'capture': not self.display_only and self.mirror is None, 'hardware': False, 'mirror': True},
                    'physics_tracking_validated': False}

    def expire_mirror(self):
        if self.mirror is not None and time.monotonic()-self.mirror['received_at'] > 2.:
            self.mirror = None
            self.status, self.reason = 'READY', 'real feedback unavailable; independent simulation'

    def mirror_feedback(self, body):
        with self.lock:
            if body.get('clear') is True:
                self.mirror = None
                if self.status == 'MIRRORING':
                    self.status, self.reason = 'READY', None
                return self.state()
            q = self.vector(body.get('q'))
            gaps = jaw_gaps(body.get('jaw_gap_mm', self.jaw_mm))
            source, sequence, age = body.get('source_id'), body.get('sequence'), body.get('source_age_s')
            if not isinstance(source, str) or not 1 <= len(source) <= 128:
                raise ValueError('mirror source_id required (1..128 characters)')
            if type(sequence) is not int or not 0 <= sequence <= 2**53:
                raise ValueError('mirror sequence must be a nonnegative integer')
            if type(age) not in (int, float) or not np.isfinite(age) or not 0 <= age <= 1.:
                raise ValueError('mirror feedback must be at most 1 second old')
            if sequence <= self.mirror_sequences.get(source, -1):
                raise ValueError('mirror feedback sequence did not advance')
            if self.mirror is None:
                if self.pending_wrist_config is not None:
                    self.clear_wrist_views()
                self.pending_wrist_config = None
                self.epoch += 1
                if self.active:
                    self.finish(stopped=True)
                self.pending = None
                self.stopped.clear()
            # ponytail: remember the last 32 source incarnations; reconnect uses a new source_id.
            self.mirror_sequences[source] = sequence
            if len(self.mirror_sequences) > 32:
                del self.mirror_sequences[next(iter(self.mirror_sequences))]
            self.mirror = {'source_id': source, 'sequence': sequence, 'source_age_s': age,
                           'received_at': time.monotonic()}
            self.q, self.jaw_mm = q, gaps
            self.status, self.reason = 'MIRRORING', None
            return self.state()

    def command(self, endpoint, body):
        if not isinstance(body, dict):
            raise ValueError('JSON object required')
        if self.display_only and endpoint not in ('/preview','/overview-view','/display-view','/mirror','/overlay-rendering','/stop'):
            raise ValueError('dedicated target renderer has no execute/capture/configuration authority')
        if endpoint == '/display-view':
            if not self.display_only or set(body)!={'view'} or body['view'] not in ('target','overview'):
                raise ValueError('explicit display-only view subscription required')
            with self.lock:
                self.active_display_view=body['view']
                if body['view']!='target':self.preview=None
            return {'active_display_view':self.active_display_view}
        if endpoint == '/overlay-rendering':
            if set(body)!={'enabled'} or type(body['enabled']) is not bool:raise ValueError('enabled must be boolean')
            with self.lock:
                self.mesh_rendering_enabled=body['enabled']
                # Invalidate in-flight publications without discarding selected transforms.
                self.calibration_revision+=1;self.wrist_overlay_revision+=1
                self.calibration_state={**self.calibration_state,'ready':False}
                for value in self.wrist_overlay_state.values():value['ready']=False
            return {'enabled':self.mesh_rendering_enabled}
        if endpoint == '/overview-view':
            view=validate_overview_view(body)
            with self.lock:
                revision=body.get('revision',self.overview_requested_revision+1)
                if type(revision) is not int or not 0<=revision<=2**53:raise ValueError('invalid overview revision')
                if revision>self.overview_requested_revision:
                    self.overview_requested_revision=revision
                    self.overview_view=(view,revision)
            return {'queued':True,'overview_revision':revision}
        if endpoint == '/mirror':
            return self.mirror_feedback(body)
        if endpoint == '/wrist-config':
            contract = validate_wrist_contract(body['wrist_contract'])
            if contract.get('marker_size_m', self.geometry.marker_size) != self.geometry.marker_size:
                raise ValueError('wrist marker size differs from rendered sticker size')
            with self.lock:
                self.expire_mirror()
                if self.status not in ('READY','MIRRORING') or self.active or self.validation or self.pending is not None or self.stopped.is_set():
                    raise ValueError('wrist display configuration requires idle READY or MIRRORING renderer')
                self.clear_wrist_views()
                self.pending_wrist_config = contract
            return {'queued': True, 'contract_sha256': contract_sha256(contract)}
        if endpoint == '/wrist-calibration':
            observer = body.get('observer', body.get('side'))
            if observer not in ('left', 'right'):
                raise ValueError('wrist observer must be left/right')
            with self.lock:
                if body.get('clear') is True:
                    ident = None
                    if observer not in self.wrist_calibrations and self.wrist_overlay_state.get(observer, {}).get('id') is None:
                        self.wrist_overlay_state[observer] = {'id': None, 'ready': False}
                        return self.state()
                    self.wrist_calibrations.pop(observer, None)
                else:
                    contract = self.pending_wrist_config or self.wrist_contract
                    if contract is None:
                        raise ValueError('wrist capture contract required before overlay')
                    digest = body.get('head_snapshot_sha256', body.get('source_snapshot_sha256'))
                    if digest != contract['head_snapshot']['snapshot_sha256']:
                        raise ValueError('wrist overlay head snapshot mismatch')
                    if body.get('contract_sha256') != contract_sha256(contract):
                        raise ValueError('wrist overlay full contract mismatch')
                    ident = body.get('id')
                    if not isinstance(ident, str) or not 1 <= len(ident) <= 128:
                        raise ValueError('wrist calibration id required')
                    transform = rigid_transform(body['T_link6_camera'])
                    previous = self.wrist_calibrations.get(observer)
                    if previous and previous['id'] == ident and np.array_equal(previous['T_link6_camera'], transform):
                        return self.state()
                    self.wrist_calibrations[observer] = {'id': ident, 'T_link6_camera': transform}
                self.wrist_overlay_revision += 1
                self.jpeg.pop(observer+'-calibration', None)
                self.jpeg.pop(observer+'-calibration-original', None)
                self.frame_timestamp.pop(observer+'-calibration', None)
                self.frame_timestamp.pop(observer+'-calibration-original', None)
                self.wrist_overlay_state[observer] = {'id': ident, 'ready': False, 'contract_sha256': contract_sha256(self.pending_wrist_config or self.wrist_contract) if (self.pending_wrist_config or self.wrist_contract) else None}
            return self.state()
        if endpoint == '/calibration':
            candidate = None if body.get('clear') is True else validate_calibration(body)
            with self.lock:
                self.calibration = candidate
                self.calibration_revision += 1
                self.jpeg.pop('calibration', None)
                self.jpeg.pop('calibration-original', None)
                self.frame_timestamp.pop('calibration', None)
                self.calibration_state = {'id': candidate['id'] if candidate else None,
                                          'ready': False, 'error': None}
            return self.state()
        if endpoint == '/stop':
            with self.lock:
                self.epoch += 1
                self.stopped.set()
                self.pending = None
                self.pending_wrist_config = None
                self.status, self.reason = 'STOPPED', 'operator stop'
            return self.state()
        if endpoint == '/ack':
            with self.lock:
                if self.active or self.validation or self.status not in ('STOPPED', 'ERROR'):
                    raise ValueError('ack requires an idle stopped/error session')
                self.stopped.clear()
                self.status, self.reason = 'READY', None
            return self.state()
        if endpoint == '/preview':
            q = self.vector(body['q'])
            gaps = jaw_gaps(body.get('jaw_gap_mm',self.jaw_mm))
            revision = body.get('revision', 0)
            if type(revision) is not int or not 0 <= revision <= 2**53:
                raise ValueError('invalid preview revision')
            with self.lock:
                newest = self.preview[1] if self.preview else self.preview_revision
                if revision > newest:
                    self.preview = (q, revision, gaps)
                    self.last_preview_received = time.monotonic()
            return self.state()
        if endpoint != '/execute':
            raise ValueError('unknown command')
        trusted_seed = body.get('standard_motion_seed') == self.STANDARD_MOTION_SEED
        with self.lock:
            self.expire_mirror()
            if self.mirror is not None:
                raise ValueError('real feedback mirror active; execute through real controller')
        contract = validate_wrist_contract(body['wrist_contract']) if body.get('wrist_contract') is not None else None
        if contract is not None and contract.get('marker_size_m', self.geometry.marker_size) != self.geometry.marker_size:
            raise ValueError('wrist marker size differs from rendered sticker size')
        start = self.vector(body.get('expected_start', body.get('expected_start12')))
        rows = body.get('waypoints')
        if not isinstance(rows, list) or not 1 <= len(rows) <= 200:
            raise ValueError('expect 1..200 waypoints')
        waypoints = []
        for row in rows:
            if not isinstance(row, dict):
                raise ValueError('invalid waypoint')
            q = self.vector(row['q'])
            speeds = np.asarray(row.get('speeds', self.speed_limits * float(row.get('speed_scale', .2))), float)
            if speeds.shape != (12,) or not np.isfinite(speeds).all() or np.any(speeds <= 0) or np.any(speeds > self.speed_limits + 1e-9):
                raise ValueError('speeds must be positive and within joint limits')
            jaw_speed=row.get('jaw_speed_mm_s',10.)
            if type(jaw_speed) not in (int,float) or not np.isfinite(jaw_speed) or jaw_speed<=0:raise ValueError('positive jaw speed required')
            capture = row.get('capture', row.get('sample', True))
            split, arm = row.get('split', 'fit'), row.get('arm')
            if type(capture) is not bool or split not in ('fit', 'holdout', 'test') or arm not in (None, 'left', 'right'):
                raise ValueError('invalid capture/split/arm')
            waypoints.append({'jaw_speed_mm_s':jaw_speed,'jaw_gap_mm':jaw_gaps(row.get('jaw_gap_mm',self.jaw_mm)), 'q': q, 'speeds': speeds, 'capture': capture, 'split': split, 'arm': arm, 'transition':bool(row.get('transition',False))})
        if contract is not None:
            captured = [row for row in rows if row.get('capture', row.get('sample', True))]
            if len(captured) != len(contract['planned_frames']):
                raise ValueError('wrist planned frame count differs from capture waypoints')
            planned_index = 0
            for source, waypoint in zip(rows, waypoints):
                if waypoint['capture']:
                    planned = contract['planned_frames'][planned_index]
                    planned_index += 1
                    if any(source.get(key, source.get('id') if key == 'planned_id' else None) != planned[{'planned_id':'id'}.get(key,key)] for key in ('planned_id','observer','target','split')):
                        raise ValueError('wrist waypoint does not match its planned frame')
                    waypoint.update(planned_id=planned['id'], observer=planned['observer'], target=planned['target'])
        with self.lock:
            if self.status != 'READY' or self.stopped.is_set() or self.validation or self.active or self.pending_wrist_config is not None:
                raise ValueError('session is not READY')
            if not np.allclose(jaw_gaps(body.get('expected_jaw_gap_mm',self.jaw_mm)),self.jaw_mm,atol=.1,rtol=0):
                raise ValueError('jaw start changed; revalidate plan')
            if not np.allclose(start, self.q, atol=1e-5, rtol=0):
                raise ValueError('actual start changed; revalidate plan')
            self.status, self.reason, self.validation = ('EXECUTING', None, False) if trusted_seed else ('VALIDATING', None, True)
            epoch = self.epoch
            self.capture_id = 'live-' + uuid.uuid4().hex
            self.dataset = None
            self.progress = {'index': 0, 'total': len(waypoints), 'samples': 0,
                             'total_samples': sum(w['capture'] for w in waypoints)}
        if trusted_seed:
            with self.lock:
                self.pending = waypoints
                self.pending_wrist_contract = contract
        else:
            threading.Thread(target=self.validate_path, args=(start, waypoints, epoch, contract), daemon=True).start()
        return self.state()

    def validate_path(self, start, waypoints, epoch, contract=None):
        try:
            for row in waypoints:
                if self.stopped.is_set() or epoch != self.epoch:
                    return
                geometry=self.geometry.with_gaps(83.,83.) if hasattr(self.geometry,'with_gaps') else self.geometry
                if contract is not None:
                    geometry = marker_geometry(geometry, contract['head_snapshot'])
                    geometry = copy.copy(geometry)
                    geometry.config = {**geometry.config, 'table_z_m': contract['table_z_m'], 'table_clearance_m': contract['table_clearance_m']}
                    offsets = np.asarray([contract['head_snapshot']['joint_offsets_rad'][s] for s in ('left','right')]).reshape(12)
                    nominal_frames = geometry.frames
                    geometry.frames = lambda q, frames=nominal_frames, delta=offsets: frames(np.asarray(q)+delta)
                result = geometry.path_check(start, row['q'])
                if not result['ok']:
                    raise ValueError('path rejected: ' + json.dumps(result))
                start = row['q']
            with self.lock:
                if epoch == self.epoch and not self.stopped.is_set():
                    self.pending = waypoints
                    self.pending_wrist_contract = contract
        except Exception as exc:
            with self.lock:
                if epoch == self.epoch:
                    self.reason, self.status = str(exc), 'ERROR'
        finally:
            with self.lock:
                self.validation = False

    def finish(self, stopped=False):
        if self.batch_dir is None:
            return
        self.dataset = {'schema': 'x5-rtx-kinematic-v1', 'capture_id': self.capture_id,
                        'camera': self.camera, 'frames': self.rows,
                        'planned_frames': copy.deepcopy(self.planned_frames),
                        'physics_tracking_validated': False, 'ideal_corners_used_as_measurements': False,
                        'stopped': stopped, 'mount_chain': getattr(self.geometry, 'mount_chain', {}),
                        'mount_status': getattr(self.geometry, 'mount_status', 'unknown'),
                        'collision_method': getattr(self.geometry, 'collision_method', 'unknown'),
                        'feedback_kind': 'DSW commanded kinematic state; not motor feedback',
                        'completed_at': time.time()}
        if self.simulation_truth_reference is not None:
            self.dataset['simulation_truth'] = copy.deepcopy(self.simulation_truth_reference)
        self.dataset.update(self.asset_identity)
        if self.wrist_contract is not None:
            self.dataset.update(wrist_contract=copy.deepcopy(self.wrist_contract),
                                planned_frames=copy.deepcopy(self.wrist_contract['planned_frames']),
                                cameras={'head': self.camera, **self.wrist_contract['cameras']})
        (self.batch_dir/'dataset.json').write_text(json.dumps(self.dataset))
        completion = {'passed': not stopped, 'stopped': stopped, 'captured': len(self.rows),
                      'accepted': len(self.rows),
                      'failed': 0,
                      'marker_detected': sum(bool(row.get('marker_detected')) for row in self.rows)}
        (self.batch_dir/'completion.json').write_text(json.dumps(completion))
        self.active = None

    def clear_wrist_views(self):
        self.wrist_calibrations.clear()
        self.wrist_overlay_revision += 1
        self.wrist_overlay_state.clear()
        self.actual_cameras = {}
        for side in ('left','right'):
            for view in (side,side+'-calibration',side+'-calibration-original'):
                self.jpeg.pop(view,None)
                self.frame_timestamp.pop(view,None)

    def tick(self, renderer, dt):
        """Advance virtual time by at most 100ms; slow RTX cannot jump over motion."""
        with self.lock:
            self.updated = time.time()
            self.expire_mirror()
            if self.pending_wrist_config is not None:
                if self.status in ('READY','MIRRORING') and not self.active and not self.stopped.is_set():
                    self.wrist_contract = self.pending_wrist_config
                    self.actual_render_started = None
                self.pending_wrist_config = None
            if self.status in ('VALIDATING', 'EXECUTING') and time.monotonic()-self.client_seen > 10:
                self.epoch += 1
                self.stopped.set()
                self.status, self.reason = 'STOPPED', 'client heartbeat lost for 10 seconds'
            if self.stopped.is_set():
                if self.active:
                    self.finish(stopped=True)
                self.pending = None
            elif self.pending is not None:
                self.batch_dir = self.output/self.capture_id
                self.batch_dir.mkdir()
                (self.batch_dir/'camera.json').write_text(json.dumps(self.camera))
                if self.urdf:
                    shutil.copyfile(self.urdf, self.batch_dir/'robot.urdf')
                self.rows = []
                self.planned_frames = [{'id': f'frame-{i:03d}', 'split': row['split']}
                                       for i, row in enumerate(w for w in self.pending if w['capture'])]
                self.wrist_contract = self.pending_wrist_contract
                self.simulation_truth_reference = None
                if hasattr(renderer, 'simulation_truth'):
                    truth = renderer.simulation_truth(self.capture_id, self.wrist_contract)
                    # Separate from the measurement directory and HTTP artifact allowlist.
                    truth_dir = self.output.parent/'simulation-truth'
                    truth_dir.mkdir(exist_ok=True)
                    with (truth_dir/(self.capture_id+'.json')).open('x') as stream:
                        json.dump(truth, stream, indent=2, allow_nan=False)
                    self.simulation_truth_reference = {
                        'capture_id': self.capture_id, 'fixture_sha256': truth['fixture_sha256'],
                        'sha256': contract_sha256(truth)}
                self.clear_wrist_views()
                self.active = {'waypoints': self.pending, 'index': 0, 'elapsed': 0., 'start': self.q.copy(), 'start_jaw':self.jaw_mm.copy()}
                self.pending = None
                self.status = 'EXECUTING'
            active = self.active
            epoch = self.epoch
            q = self.q.copy()
            gaps = self.jaw_mm.copy()
            endpoint = False
            if active:
                row = active['waypoints'][active['index']]
                target_gaps=row.get('jaw_gap_mm',active['start_jaw'])
                duration = max(.1, float(np.max(1.875*abs(row['q']-active['start'])/row['speeds'])),float(np.max(1.875*abs(target_gaps-active['start_jaw'])/row.get('jaw_speed_mm_s',10.))))
                active['elapsed'] += min(max(dt, 0), .1)
                u = min(1., active['elapsed']/duration)
                q = active['start']+(row['q']-active['start'])*(10*u**3-15*u**4+6*u**5)
                gaps=active['start_jaw']+(target_gaps-active['start_jaw'])*(10*u**3-15*u**4+6*u**5)
                # Zero endpoint velocity first, then hold 300ms of simulation time.
                endpoint = u >= 1 and active['elapsed'] >= duration + .3
            # During manual dragging, reserve render ticks for the target. Camera
            # feedback remains fresh at 10 Hz; collection (no preview traffic) stays 30 Hz.
            actual_interval = .1 if time.monotonic()-self.last_preview_received < .3 else 1/30
            render_actual = active is not None or self.actual_render_started is None or time.monotonic()-self.actual_render_started >= actual_interval
            if self.display_only and self.active_display_view!='overview':render_actual=False
            preview = self.preview
            self.preview = None
        with self.lock:
            view=self.overview_view;self.overview_view=None
        if view is not None:
            renderer.set_overview_view(view[0])
            self.overview_revision=view[1]
            if preview is None and not self.display_only:preview=self.last_preview_pose
            render_actual=not self.display_only or self.active_display_view=='overview'
        overview_revision=self.overview_revision
        if self.display_only:
            if self.active_display_view!='target':preview=None
            if hasattr(renderer,'actual_updates'):renderer.actual_updates(render_actual)
            if hasattr(renderer,'preview_updates'):renderer.preview_updates(preview is not None)
        if hasattr(renderer, 'configure_wrist'):
            renderer.configure_wrist(self.wrist_contract)
        # No lock around RTX: stop and health requests remain responsive.
        paired = None
        if render_actual and preview and hasattr(renderer, 'render_pair') and not (active and endpoint and active['waypoints'][active['index']]['capture']):
            pair_started = time.monotonic()
            paired = renderer.render_pair(q, gaps, preview[0], preview[2])
        if render_actual:
            started = time.monotonic()
            self.actual_render_started = pair_started if paired is not None else started
            capture_frame = active is not None and endpoint and active['waypoints'][active['index']]['capture']
            if paired is not None:
                started = pair_started
                jpeg, head = paired[0]
            elif capture_frame and hasattr(renderer, 'render_capture'):
                jpeg, head = renderer.render_capture(q, gaps)
            else:
                jpeg, head = renderer.render(q,gaps)
            with self.lock:
                self.actual_head = head.copy() if hasattr(head, 'copy') else head
                self.jpeg['actual'] = jpeg
                self.frame_timestamp['actual'] = time.time()
                self.frame_metadata['actual']=self.display_metadata('actual',q,gaps,self.frame_timestamp['actual'],
                    self.frame_sequence+1,overview_revision)
                if 'preview' not in self.jpeg:
                    self.jpeg['preview'] = jpeg
                    self.frame_timestamp['preview'] = self.frame_timestamp['actual']
                self.actual_q, self.actual_gaps = q.copy(), gaps.copy()
                if epoch == self.epoch and self.mirror is None:
                    self.q = q
                    self.jaw_mm = gaps.copy()
                self.render_times['actual'].append(time.monotonic())
                self.render_ms['actual'] = (time.monotonic()-started)*1000
                self.render_stages['actual'] = dict(getattr(renderer, 'last_timings', {}))
                self.frame_ready.notify_all()
                self.frame_sequence += 1
                if hasattr(renderer, 'encode_head'):
                    self.jpeg['head'] = renderer.encode_head(head)
                    self.frame_timestamp['head'] = self.frame_timestamp['actual']
                self.actual_cameras = dict(getattr(renderer, 'last_camera_images', {}))
                for side in ('left', 'right'):
                    if side in self.actual_cameras:
                        self.jpeg[side] = renderer.encode_head(self.actual_cameras[side])
                        self.frame_timestamp[side] = self.frame_timestamp['actual']
                self.updated = time.time()
                if self.status == 'STARTING':
                    self.status = 'READY'
                if active and endpoint and epoch == self.epoch and not self.stopped.is_set():
                    row = active['waypoints'][active['index']]
                    if row['capture']:
                        capture_row = {**row, 'frame_sequence': self.frame_sequence, 'timestamp_s': self.frame_timestamp['actual']}
                        record = renderer.capture(head, q, capture_row, self.batch_dir, len(self.rows))
                        self.rows.append(record)
                        self.progress['samples'] = len(self.rows)
                    active['index'] += 1
                    self.progress['index'] = active['index']
                    if active['index'] == len(active['waypoints']):
                        self.finish()
                        self.status = 'READY'
                    else:
                        active['start'], active['elapsed'] = q.copy(), 0.
                        active['start_jaw']=gaps.copy()
        if preview:
            started = time.monotonic()
            if paired is not None:
                started = pair_started
                jpeg, _ = paired[1]
            else:
                jpeg, _ = renderer.render(preview[0],preview[2], preview=True) if hasattr(renderer, 'render_pair') else renderer.render(preview[0],preview[2])
            with self.lock:
                self.jpeg['preview'] = jpeg
                self.frame_timestamp['preview'] = time.time()
                self.preview_revision = preview[1]
                self.last_preview_pose=preview
                self.frame_metadata['preview']=self.display_metadata('preview',preview[0],preview[2],
                    self.frame_timestamp['preview'],preview[1],overview_revision)
                self.render_times['preview'].append(time.monotonic())
                self.render_ms['preview'] = (time.monotonic()-started)*1000
                self.render_stages['preview'] = dict(getattr(renderer, 'last_timings', {}))
                self.frame_ready.notify_all()
                self.updated = time.time()

        with self.lock:
            candidate = self.calibration
            revision = self.calibration_revision
            sequence = self.frame_sequence
            actual_q = self.actual_q.copy()
            actual_gaps = self.actual_gaps.copy()
            head = self.actual_head
            overlay_interval = .5 if time.monotonic()-self.last_preview_received < .3 else 0.
            needs_overlay = self.mesh_rendering_enabled and time.monotonic()-self.last_overlay_started >= overlay_interval and not self.calibration_busy and candidate is not None and head is not None and (
                self.calibration_state.get('revision') != revision or
                self.calibration_state.get('frame_sequence') != sequence)
            original = self.jpeg.get('head')
            timestamp = self.frame_timestamp.get('actual')
            if needs_overlay:
                self.calibration_busy = True
                self.last_overlay_started = time.monotonic()
        if needs_overlay:
            # Projection touches numpy/OpenCV only, never USD; keep motion/stop responsive.
            threading.Thread(target=self.update_calibration,
                args=(renderer, head, actual_q, {**candidate,'jaw_gap_mm':actual_gaps.tolist()}, revision, sequence, original, timestamp),
                daemon=True).start()
        with self.lock:
            wrist_work = bool(self.mesh_rendering_enabled and self.wrist_contract and self.wrist_calibrations and self.actual_cameras and not self.wrist_overlay_busy and render_actual
                              and time.monotonic()-self.last_wrist_overlay_started >= overlay_interval)
            if wrist_work:
                self.wrist_overlay_busy = True
                self.last_wrist_overlay_started = time.monotonic()
                wrist_args = (renderer, dict(self.actual_cameras), self.actual_q.copy(), self.actual_gaps.copy(),
                              copy.deepcopy(self.wrist_calibrations), self.wrist_contract,
                              self.wrist_overlay_revision, self.frame_sequence, self.frame_timestamp['actual'])
        if wrist_work:
            threading.Thread(target=self.update_wrist_calibration, args=wrist_args, daemon=True).start()
        return render_actual or preview is not None

    def display_metadata(self,view,q,gaps,timestamp,sequence,overview_revision):
        stamp=int(timestamp*1_000_000_000)
        return dict(schema='target_preview_frame.v1',renderer_instance_id=self.renderer_instance_id,
            view=view,source='kinematic_target' if view=='preview' else 'kinematic_overview',
            target_revision=sequence if view=='preview' else None,overview_revision=overview_revision,
            frame_id=f'{self.renderer_instance_id}:{view}:{sequence}:{stamp}',source_timestamp_ns=stamp,
            timestamp_basis='renderer host wall publication; not an actual camera observation',
            q_reported_rad=q.tolist(),jaw_gap_mm=gaps.tolist(),display_only=True,measurement_qualified=False)

    def update_wrist_calibration(self, renderer, images, q, gaps, candidates, contract, revision, sequence, timestamp):
        results = {}
        for side, candidate in candidates.items():
            try:
                overlay = renderer.wrist_calibration_overlay(images[side], q, gaps, side, candidate, contract)
                results[side] = (overlay, None)
            except Exception as exc:
                results[side] = (None, str(exc))
        with self.lock:
            self.wrist_overlay_busy = False
            if revision != self.wrist_overlay_revision:
                return
            for side, (overlay, error) in results.items():
                self.wrist_overlay_state[side] = {'id': candidates[side]['id'], 'ready': overlay is not None,
                    'error': error, 'frame_sequence': sequence, 'timestamp_s': timestamp,
                    'head_snapshot_sha256': contract['head_snapshot']['snapshot_sha256'],
                    'contract_sha256': contract_sha256(contract)}
                if overlay is not None:
                    self.jpeg[side+'-calibration'] = overlay
                    self.jpeg[side+'-calibration-original'] = renderer.encode_head(images[side])
                    self.frame_timestamp[side+'-calibration'] = timestamp
                    self.frame_timestamp[side+'-calibration-original'] = timestamp

    def update_calibration(self, renderer, head, q, candidate, revision, sequence, original, timestamp):
        error, overlay = None, None
        try:
            overlay = renderer.calibration_overlay(head, q, candidate)
        except Exception as exc:
            error = str(exc)
        with self.lock:
            self.calibration_busy = False
            if revision == self.calibration_revision:
                self.calibration_state = {'id': candidate['id'], 'revision': revision,
                    'frame_sequence': sequence, 'q': q.tolist(), 'timestamp_s': timestamp,
                    'rendered_at_s': time.time(), 'ready': overlay is not None, 'error': error,
                    'calibration_frame_id': str(revision)+':'+str(sequence) if overlay is not None else None}
                if overlay is not None:
                    self.jpeg['calibration'] = overlay
                    if original is not None:
                        self.jpeg['calibration-original'] = original
                    self.frame_timestamp['calibration'] = timestamp


def rigid_transform(value):
    from arx_env.joint_head_isaac_fit import rigid
    return rigid(value)


def marker_geometry(geometry, snapshot):
    mounts = {side:rigid_transform(snapshot['T_link6_marker'][marker]) for side,marker in (('left','24'),('right','30'))}
    view = geometry.with_marker_mounts(mounts)
    if view is not geometry:
        # Geometry views are shallow copies. Keeping the previous overlay here
        # retains every replaced GPU cache through overlay.geometry -> old cache.
        for name in ('_wrist_gpu_overlay_cache', '_real_gpu_overlay'):
            view.__dict__.pop(name, None)
    return view


def simulation_wrist_mounts(config):
    """Renderer-only fixture; never copied to measurement contracts or HTTP datasets."""
    mounts = config.get('wrist_simulation_mounts', {})
    if not isinstance(mounts,dict) or any(side not in ('left','right') for side in mounts):
        raise ValueError('wrist simulation mounts require left/right rigid transforms')
    return {side:rigid_transform(value).copy() for side,value in mounts.items()}


def contract_sha256(contract):
    return hashlib.sha256(json.dumps(contract,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_wrist_contract(body):
    """Freeze a JSON-only measurement contract; hidden simulation truth is never read."""
    contract = json.loads(json.dumps(body, allow_nan=False))
    if not isinstance(contract,dict) or any(not isinstance(contract.get(k),dict) for k in ('head_snapshot','cameras','nominal_mounts')):
        raise ValueError('wrist contract requires head_snapshot, cameras and nominal_mounts objects')
    head = contract['head_snapshot']
    validate_calibration(head)
    if head.get('snapshot_complete') is not True or head.get('marker_ids') != {'left':24, 'right':30}:
        raise ValueError('complete frozen head/sticker snapshot required')
    for marker in ('24','30'):
        rigid_transform(head['T_link6_marker'][marker])
    payload = {key:head[key] for key in ('id','camera_T_world_optical','joint_offsets_rad','marker_ids','T_link6_marker')}
    digest = hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if head.get('snapshot_sha256') != digest:
        raise ValueError('frozen head snapshot digest mismatch')
    for side in ('left','right'):
        rigid_transform(contract['nominal_mounts'][side])
        camera = contract['cameras'][side]
        if not isinstance(camera,dict):
            raise ValueError('wrist camera intrinsics must be an object')
        values = np.asarray([camera[k] for k in ('fx','fy','cx','cy')], float)
        coefficients = np.asarray(camera.get('coefficients',[0.]*5),float)
        if (not np.isfinite(values).all() or np.any(values[:2] <= 0) or
                coefficients.shape != (5,) or not np.isfinite(coefficients).all() or
                any(type(camera[k]) is not int or not 1 <= camera[k] <= 4096 for k in ('width','height')) or
                camera.get('rectified') is not True or np.any(coefficients)):
            raise ValueError('finite rectified wrist pinhole intrinsics required')
    planned = contract['planned_frames']
    if not isinstance(planned,list) or not 1 <= len(planned) <= 200:
        raise ValueError('wrist planned frames required')
    ids = set()
    for row in planned:
        if not isinstance(row,dict):
            raise ValueError('wrist planned frame must be an object')
        if (not isinstance(row.get('id'),str) or not 1 <= len(row['id']) <= 128 or row['id'] in ids or
                row.get('observer') not in ('left','right') or row.get('target') != {'left':'right','right':'left'}[row['observer']] or
                row.get('split') not in ('fit','holdout')):
            raise ValueError('invalid wrist planned frame')
        ids.add(row['id'])
    if contract.get('table_z_m') != 0 or contract.get('table_clearance_m') != .005:
        raise ValueError('wrist capture requires table z=0 and 5 mm clearance')
    return contract


def validate_calibration(body):
    ident = body.get('id')
    if not isinstance(ident, str) or not 1 <= len(ident) <= 128:
        raise ValueError('calibration id required (1..128 characters)')
    T = np.asarray(body.get('camera_T_world_optical'), float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or
            not np.allclose(T[3], [0,0,0,1], atol=1e-7, rtol=0) or
            not np.allclose(T[:3,:3].T@T[:3,:3], np.eye(3), atol=1e-6, rtol=0) or
            not np.isclose(np.linalg.det(T[:3,:3]), 1., atol=1e-6, rtol=0)):
        raise ValueError('calibration camera must be a finite rigid transform')
    offsets = body.get('joint_offsets_rad')
    if not isinstance(offsets, dict):
        raise ValueError('left/right joint offsets required')
    values = []
    for side in ('left', 'right'):
        value = np.asarray(offsets.get(side), float)
        if value.shape != (6,) or not np.isfinite(value).all() or np.any(abs(value) > np.pi):
            raise ValueError('six finite joint offsets per arm bounded by pi required')
        values.extend(value.tolist())
    return {'id': ident, 'camera_T_world_optical': T.copy(), 'offsets': np.asarray(values)}




def validate_overview_view(body):
    values=np.asarray([body.get(k) for k in ('yaw','pitch','zoom')],float)
    if not np.isfinite(values).all() or not -np.pi<=values[0]<=np.pi or not -.15<=values[1]<=1.5 or not .4<=values[2]<=4:
        raise ValueError('invalid overview camera')
    pan=float(body.get('pan_y',0.))
    if not np.isfinite(pan) or abs(pan)>2:raise ValueError('invalid overview pan')
    return {**dict(zip(('yaw','pitch','zoom'),values.tolist())),'pan_y':pan}


def overview_camera(geometry, view=None):
    """Fixed overview fitted to full initial mesh bounds with 20% screen margin."""
    points=getattr(geometry,'_overview_bounds',None)
    if points is None:
        frames = geometry.frames(geometry.config.get('initial_q', [0.]*12))
        points = []
        for side, meshes in geometry.side_meshes.items():
            for name, vertices, _ in meshes:
                T = frames[side][name]
                points.append(vertices@T[:3,:3].T+T[:3,3])
        points.extend(vertices for _,vertices,_ in getattr(geometry, 'static_meshes', []))
        points = np.concatenate(points)
        low,high=points.min(0),points.max(0)
        points=np.array([[x,y,z] for x in (low[0],high[0]) for y in (low[1],high[1]) for z in (low[2],high[2])])
        geometry._overview_bounds=points
    center = (points.min(0)+points.max(0))/2
    view=view or {'yaw':-np.pi/2,'pitch':.4,'zoom':1.}
    yaw,pitch=view['yaw'],view['pitch']
    z = -np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),np.sin(pitch)])
    x = np.cross(z, [0,0,1]); x /= np.linalg.norm(x)
    rotation = np.column_stack((x, np.cross(z,x), z))
    local = (points-center)@rotation
    # z(camera) = distance + local z; solve the two pinhole framing bounds.
    distance = max(.1, float(np.max(np.maximum(abs(local[:,0])*640/(480*.8),
                                                abs(local[:,1])*640/(360*.8))-local[:,2])))
    pose = np.eye(4); pose[:3,:3] = rotation; pose[:3,3] = center-z*distance/view['zoom']-rotation[:,1]*view.get('pan_y',0.)*distance*.6
    return {'fx': 640., 'fy': 640., 'cx': 480., 'cy': 360., 'width': 960,
            'height': 720, 'T_world_optical': pose}


class RTXRenderer:
    def __init__(self, geometry):
        import cv2
        from pxr import Gf, UsdGeom, UsdShade, UsdLux, Sdf, Vt
        import omni.usd
        import omni.replicator.core as rep
        self.g, self.cv2, self.Gf, self.rep = geometry, cv2, Gf, rep
        self.simulation_wrist_mounts = simulation_wrist_mounts(geometry.config)
        # Freeze the scene fixture at startup; request/fit parameters never set truth.
        for side, value in geometry.config.get('wrist_nominal_mounts', {}).items():
            self.simulation_wrist_mounts.setdefault(side, rigid_transform(value).copy())
        offsets = geometry.config.get('simulation_joint_offsets_rad', {s:[0.]*6 for s in ('left','right')})
        self.simulation_joint_offsets = np.asarray([offsets[s] for s in ('left','right')], float)
        if self.simulation_joint_offsets.shape != (2,6) or not np.isfinite(self.simulation_joint_offsets).all():
            raise ValueError('twelve finite simulation joint offsets required')
        self.simulation_joint_offsets = self.simulation_joint_offsets.reshape(12)
        self.wrist_geometry = geometry
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        mats = []
        for i, color in enumerate(((.008,)*3, (.95,)*3, (.26, .29, .34))):
            mat = UsdShade.Material.Define(stage, f'/World/Materials/m{i}')
            shader = UsdShade.Shader.Define(stage, f'/World/Materials/m{i}/Shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.85)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            mats.append(mat)
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1200.)

        def mesh(path, vertices, faces, material):
            m = UsdGeom.Mesh.Define(stage, path)
            m.CreatePointsAttr(np.asarray(vertices).tolist())
            m.CreateFaceVertexCountsAttr([len(f) for f in faces])
            m.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
            m.CreateDoubleSidedAttr(True)
            m.CreateSubdivisionSchemeAttr('none')
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
            return m.AddTransformOp()

        UsdGeom.Xform.Define(stage, '/World/Chassis')
        for name, vertices, faces in getattr(geometry, 'static_meshes', []):
            mesh('/World/Chassis/'+name, vertices, faces, mats[2])
        self.parts = []
        for side, meshes in geometry.side_meshes.items():
            for name, vertices, _ in meshes:
                # Geometry stores original triangle soup; sampled display faces are not used.
                op = mesh(f'/World/Robot/{side}/{name}', vertices, np.arange(len(vertices)).reshape(-1, 3), mats[2])
                self.parts.append((side, name, op))
        self.Vt=Vt
        self.stage=stage
        self.UsdGeom=UsdGeom
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.tags = []
        for side, ident in [('left', 24), ('right', 30)]:
            root = f'/World/Tags/{side}'
            op = UsdGeom.Xform.Define(stage, root).AddTransformOp()
            self.tags.append((side, op))
            pattern = np.pad(cv2.aruco.generateImageMarker(dictionary, ident, 6), 1, constant_values=255)
            size = geometry.marker_size
            for y in range(8):
                for x in range(8):
                    a, b = (x-1)/6-.5, (y-1)/6-.5
                    vertices = [[u*size, v*size, 0] for u, v in ((a,b),(a+1/6,b),(a+1/6,b+1/6),(a,b+1/6))]
                    mesh(root+f'/Cell{x}_{y}', vertices, [[0,1,2,3]], mats[int(pattern[y,x] > 0)])
        self.annotators = []
        self.actual_products = []
        self.preview_products = []
        self.pose_cache = {}
        self.wrist_contract = None
        self.wrist_annotators = {}
        self.wrist_products = {}
        self.wrist_camera_ops = {}
        self.wrist_camera_specs = {}
        self.wrist_warmup = False
        self.last_camera_images = {}
        head = {**geometry.k, 'T_world_optical': geometry.camera}
        overview = overview_camera(geometry)
        # Overview is a display image; calibrated head pixels keep their original resolution.
        scale = 2/3
        for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height'):
            overview[key] *= scale
        for name, c in [('Head', head), ('Overview', overview)]:
            cam = UsdGeom.Camera.Define(stage, '/World/'+name)
            pose = np.asarray(c['T_world_optical'])@np.diag([1,-1,-1,1])
            cam.AddTransformOp().Set(Gf.Matrix4d(*pose.T.ravel().tolist()))
            unit = .01
            cam.CreateFocalLengthAttr(c['fx']*unit)
            cam.CreateHorizontalApertureAttr(c['width']*unit)
            cam.CreateVerticalApertureAttr(c['height']*unit*c['fx']/c['fy'])
            cam.CreateHorizontalApertureOffsetAttr((c['width']/2-c['cx'])*unit)
            cam.CreateVerticalApertureOffsetAttr((c['cy']-c['height']/2)*unit*c['fx']/c['fy'])
            cam.CreateClippingRangeAttr(Gf.Vec2f(.005, 20.))
            product = rep.create.render_product(str(cam.GetPath()), (int(c['width']), int(c['height'])))
            rgb = rep.AnnotatorRegistry.get_annotator('rgb'); rgb.attach([product])
            self.annotators.append(rgb)
            self.actual_products.append(product)
        # Separate translated scene prevents a target preview from changing actual feedback imagery.
        UsdGeom.Xform.Define(stage, '/Preview').AddTranslateOp().Set(Gf.Vec3d(30., 0., 0.))
        layer = stage.GetRootLayer()
        for name in ('Robot', 'Tags', 'Head', 'Overview', 'Chassis'):
            if not Sdf.CopySpec(layer, '/World/'+name, layer, '/Preview/'+name):
                raise RuntimeError('preview scene copy failed: '+name)
        self.preview_parts = [(side, name, UsdGeom.Xformable(stage.GetPrimAtPath('/Preview/Robot/'+side+'/'+name)).GetOrderedXformOps()[0]) for side,name,_ in self.parts]
        self.preview_tags = [(side, UsdGeom.Xformable(stage.GetPrimAtPath('/Preview/Tags/'+side)).GetOrderedXformOps()[0]) for side,_ in self.tags]
        self.preview_annotators = []
        for name,c in [('Head',head), ('Overview',overview)]:
            product = rep.create.render_product('/Preview/'+name, (int(c['width']), int(c['height'])))
            rgb = rep.AnnotatorRegistry.get_annotator('rgb'); rgb.attach([product])
            self.preview_annotators.append(rgb)
            self.preview_products.append(product)
        for _ in range(4):
            rep.orchestrator.step(rt_subframes=2)
        self.preview_updates(False)

    def configure_wrist(self, contract):
        # Called from tick only: HTTP threads never create/move USD cameras.
        if contract == self.wrist_contract:
            return
        if contract is not None and set(self.simulation_wrist_mounts) != {'left','right'}:
            raise ValueError('fixed renderer wrist fixture required; candidate mounts cannot define observations')
        self.wrist_contract = copy.deepcopy(contract)
        self.pose_cache.clear()
        self.last_camera_images = {}
        if contract is None:
            for product in self.wrist_products.values():
                product.hydra_texture.set_updates_enabled(False)
            return
        for side in ('left','right'):
            k = contract['cameras'][side]
            if self.wrist_camera_specs.get(side) != k:
                if side in self.wrist_products:
                    self.wrist_annotators[side].detach([self.wrist_products[side]])
                    self.wrist_products[side].destroy()
                path = '/World/Wrist'+side.title()
                cam = self.UsdGeom.Camera.Define(self.stage, path)
                ops = cam.GetOrderedXformOps()
                self.wrist_camera_ops[side] = ops[0] if ops else cam.AddTransformOp()
                unit = .01
                cam.CreateFocalLengthAttr(k['fx']*unit)
                cam.CreateHorizontalApertureAttr(k['width']*unit)
                cam.CreateVerticalApertureAttr(k['height']*unit*k['fx']/k['fy'])
                cam.CreateHorizontalApertureOffsetAttr((k['width']/2-k['cx'])*unit)
                cam.CreateVerticalApertureOffsetAttr((k['cy']-k['height']/2)*unit*k['fx']/k['fy'])
                cam.CreateClippingRangeAttr(self.Gf.Vec2f(.005,20.))
                product = self.rep.create.render_product(path,(k['width'],k['height']))
                rgb = self.rep.AnnotatorRegistry.get_annotator('rgb')
                rgb.attach([product])
                self.wrist_products[side], self.wrist_annotators[side] = product,rgb
                self.wrist_camera_specs[side] = copy.deepcopy(k)
                self.wrist_warmup = True
            self.wrist_products[side].hydra_texture.set_updates_enabled(True)

    def set_overview_view(self, view):
        camera=overview_camera(self.g,view)
        pose=np.asarray(camera['T_world_optical'])@np.diag([1,-1,-1,1])
        for root in ('/World','/Preview'):
            op=self.UsdGeom.Xformable(self.stage.GetPrimAtPath(root+'/Overview')).GetOrderedXformOps()[0]
            op.Set(self.Gf.Matrix4d(*pose.T.ravel().tolist()))

    def actual_updates(self, enabled):
        for product in [*self.actual_products, *self.wrist_products.values()]:
            product.hydra_texture.set_updates_enabled(enabled)

    def preview_updates(self, enabled):
        for product in self.preview_products:
            product.hydra_texture.set_updates_enabled(enabled)

    def pose(self, q, gaps, preview=False):
        root = '/Preview' if preview else '/World'
        q = np.asarray(q)+self.simulation_joint_offsets
        last_q, last_gaps = self.pose_cache.get(root, (None, None))
        gap_values = jaw_gaps(gaps)
        articulated = getattr(self.g, 'hand', None) is None and hasattr(self.wrist_geometry, 'with_gaps')
        for i,(side,gap) in enumerate(zip(('left','right'),gap_values)):
            if articulated or (last_gaps is not None and gap == last_gaps[i]):
                continue
            vertices = self.g.hand['vertices'].copy()
            vertices[:,1] += self.g.hand['signs']*(gap/1000-self.g.hand['nominal_gap_m'])/2
            self.UsdGeom.Mesh.Get(self.stage,root+'/Robot/'+side+'/link6').GetPointsAttr().Set(self.Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(vertices, dtype=np.float32)))
        if last_q is None or not np.array_equal(q,last_q) or (articulated and not np.array_equal(gap_values,last_gaps)):
            geometry = self.wrist_geometry.with_gaps(*gap_values) if articulated else self.wrist_geometry
            frames = geometry.frames(q)
            changed={side for i,side in enumerate(('left','right'))
                     if last_q is None or not np.array_equal(q[i*6:i*6+6],last_q[i*6:i*6+6])
                     or last_gaps is None or gap_values[i]!=last_gaps[i]}
            for side,name,op in (self.preview_parts if preview else self.parts):
                if side in changed:op.Set(self.Gf.Matrix4d(*frames[side][name].T.ravel().tolist()))
            for side,op in (self.preview_tags if preview else self.tags):
                if side not in changed:continue
                mount = self.g.mounts[side]
                pose = frames[side]['link6']@mount
                op.Set(self.Gf.Matrix4d(*pose.T.ravel().tolist()))
            if self.wrist_contract is not None and not preview:
                for side, op in self.wrist_camera_ops.items():
                    if side not in changed:continue
                    mount = self.simulation_wrist_mounts[side]
                    pose = frames[side]['link6']@np.asarray(mount)@np.diag([1,-1,-1,1])
                    op.Set(self.Gf.Matrix4d(*pose.T.ravel().tolist()))
        self.pose_cache[root] = np.asarray(q).copy(), gap_values.copy()

    def simulation_truth(self, capture_id, contract):
        """Private post-fit scoring evidence, never included in solver measurements."""
        digest = hashlib.sha256(Path(self.g.config['urdf']).read_bytes())
        for side in ('left','right'):
            digest.update(np.asarray(self.g.bases[side], dtype='<f8').tobytes())
            for name, vertices, faces in self.g.side_meshes[side]:
                digest.update(name.encode());digest.update(np.asarray(vertices,dtype='<f8').tobytes())
        for name, vertices, faces in self.g.static_meshes:
            digest.update(name.encode());digest.update(np.asarray(vertices,dtype='<f8').tobytes())
        value = dict(schema='x5-renderer-truth-v1', capture_id=capture_id,
            head=dict(camera_T_world_optical=self.g.camera.tolist(),
                joint_offsets_rad={s:self.simulation_joint_offsets[i*6:i*6+6].tolist() for i,s in enumerate(('left','right'))},
                T_link6_marker={str(i):self.g.mounts[s].tolist() for s,i in (('left',24),('right',30))}),
            wrists={s:T.tolist() for s,T in self.simulation_wrist_mounts.items()},
            intrinsics=copy.deepcopy(dict(head=self.g.k, **({s:contract['cameras'][s] for s in ('left','right')} if contract else {}))),
            marker_size_m=self.g.marker_size, geometry_sha256=digest.hexdigest(),
            renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            jaw_nominal_gap_m=self.g.hand['nominal_gap_m'] if self.g.hand is not None else None)
        value['fixture_sha256']=contract_sha256({key:item for key,item in value.items() if key != 'capture_id'})
        return value

    def read_frame(self, preview=False):
        head, overview = [np.asarray(a.get_data())[...,:3].copy() for a in (self.preview_annotators if preview else self.annotators)]
        if not preview:
            self.last_camera_images = {'head': head}
            if self.wrist_contract is not None:
                for side, annotator in self.wrist_annotators.items():
                    frame = np.asarray(annotator.get_data())[...,:3].copy()
                    k = self.wrist_contract['cameras'][side]
                    if frame.shape != (k['height'], k['width'], 3):
                        raise RuntimeError('wrist RTX frame is not ready: '+side)
                    self.last_camera_images[side] = frame
        h = overview.shape[0]
        inset = self.cv2.resize(head, (int(head.shape[1]*h/head.shape[0]), h))
        combined = np.concatenate([overview, inset], axis=1) if preview else overview
        ok,data = self.cv2.imencode('.jpg', self.cv2.cvtColor(combined,self.cv2.COLOR_RGB2BGR), [self.cv2.IMWRITE_JPEG_QUALITY,80])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        return data.tobytes(),head

    def step(self, subframes):
        if self.wrist_warmup:
            for _ in range(3):
                self.rep.orchestrator.step(rt_subframes=2)
            self.wrist_warmup = False
        self.rep.orchestrator.step(rt_subframes=subframes)

    def render_capture(self, q, gaps):
        return self.render(q,gaps,subframes=2)

    def render(self, q, gaps, subframes=1, preview=False):
        started = time.perf_counter()
        self.pose(q,gaps,preview)
        self.actual_updates(not preview)
        self.preview_updates(preview)
        posed = time.perf_counter()
        self.step(subframes)
        rendered = time.perf_counter()
        frame = self.read_frame(preview)
        self.last_timings = {'pose':(posed-started)*1000, 'rtx':(rendered-posed)*1000,
                             'readback_jpeg':(time.perf_counter()-rendered)*1000}
        return frame

    def render_pair(self, q, gaps, preview_q, preview_gaps):
        started = time.perf_counter()
        self.pose(q,gaps)
        self.pose(preview_q,preview_gaps,True)
        self.actual_updates(True)
        self.preview_updates(True)
        posed = time.perf_counter()
        self.step(1)
        rendered = time.perf_counter()
        result = self.read_frame(),self.read_frame(True)
        self.last_timings = {'pose':(posed-started)*1000, 'rtx':(rendered-posed)*1000,
                             'readback_jpeg':(time.perf_counter()-rendered)*1000, 'batched_views':2}
        return result

    def encode_head(self, head):
        ok, encoded = self.cv2.imencode('.jpg', self.cv2.cvtColor(head, self.cv2.COLOR_RGB2BGR), [self.cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            raise RuntimeError('head JPEG encoding failed')
        return encoded.tobytes()

    def calibration_overlay(self, image, q, candidate):
        from arx_env.joint_head_workbench_overlay import mesh_overlay
        overlay = mesh_overlay(self.g, image, q, candidate)
        ok, encoded = self.cv2.imencode('.jpg', self.cv2.cvtColor(overlay, self.cv2.COLOR_RGB2BGR), [self.cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            raise RuntimeError('calibration JPEG encoding failed')
        return encoded.tobytes()

    def wrist_calibration_overlay(self, image, q, gaps, side, candidate, contract):
        from arx_env.joint_head_gpu_overlay import GPUOverlay
        from arx_env.joint_head_jaw_mapping import feedback_sdk
        snapshot = contract['head_snapshot']
        key = contract_sha256(snapshot['T_link6_marker'])
        cache = getattr(self, '_wrist_gpu_overlay_cache', None)
        if cache is None or cache[0] is not self.g or cache[1] != key:
            geometry = marker_geometry(self.g, snapshot)
            cache = (self.g, key, geometry, GPUOverlay(geometry))
            self._wrist_gpu_overlay_cache = cache
        geometry, overlay = cache[2:]
        q = np.asarray(q, float)
        if q.shape != (12,) or not np.isfinite(q).all():raise ValueError('twelve finite reported joints required')
        gaps = jaw_gaps(gaps)
        offsets = np.asarray([snapshot['joint_offsets_rad'][s] for s in ('left','right')]).reshape(12)
        camera = geometry.frames(q+offsets)[side]['link6']@candidate['T_link6_camera']
        # GPUOverlay consumes SDK feedback slots; preserve the simulated physical
        # jaw gaps by applying the inverse feedback map, not the command map.
        jaws = [feedback_sdk(s, float(gap)) for s,gap in zip(('left','right'),gaps)]
        raw_q = np.r_[q[:6],jaws[0],q[6:],jaws[1]]
        rendered = overlay.render(self.cv2.cvtColor(image,self.cv2.COLOR_RGB2BGR),raw_q,
            {**snapshot,'camera_T_world_optical':camera.tolist()},contract['cameras'][side])
        ok, encoded = self.cv2.imencode('.jpg',rendered,[self.cv2.IMWRITE_JPEG_QUALITY,88])
        if not ok:raise RuntimeError('wrist GPU overlay JPEG encoding failed')
        return encoded.tobytes()

    def capture(self, image, q, waypoint, output, index):
        name = f'frame-{index:03d}'
        if not self.cv2.imwrite(str(output/(name+'.png')), self.cv2.cvtColor(image, self.cv2.COLOR_RGB2BGR)):
            raise RuntimeError('PNG write failed')
        corners, ids, _ = self.detector.detectMarkers(self.cv2.cvtColor(image, self.cv2.COLOR_RGB2GRAY))
        detections = {} if ids is None else {str(int(i)): p.reshape(4,2).tolist() for i,p in zip(ids.ravel(), corners)}
        expected = {'left': 24, 'right': 30}.get(waypoint['arm'])
        row = {'id': name, 'split': waypoint['split'], 'image': name+'.png',
               'jaw_gap_mm':jaw_gaps(waypoint.get('jaw_gap_mm',[83.,83.])).tolist(),
               'qpos': q[:6].tolist()+[float(waypoint.get('jaw_gap_mm',[83.,83.])[0])/1000]+q[6:].tolist()+[float(waypoint.get('jaw_gap_mm',[83.,83.])[1])/1000], 'q_reported_rad': q.tolist(),
               'detections': detections, 'valid_for_fit': True, 'marker_detected': str(expected) in detections if expected else bool(detections),
               'expected_marker_id': expected, 'bracket_gap_s': 0., 'pose_span_rad': [0.]*12,
               'source': 'Isaac RTX X5 persistent kinematic rendered pixels',
               'feedback_kind': 'commanded kinematic state; not motor feedback'}
        if self.wrist_contract is not None:
            camera_images, camera_detections = {'head': name+'.png'}, {'head': detections}
            for side in ('left','right'):
                pixels = self.last_camera_images[side]
                filename = name+'-'+side+'.png'
                if not self.cv2.imwrite(str(output/filename),self.cv2.cvtColor(pixels,self.cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('wrist PNG write failed: '+side)
                corners, ids, _ = self.detector.detectMarkers(self.cv2.cvtColor(pixels,self.cv2.COLOR_RGB2GRAY))
                camera_images[side] = filename
                camera_detections[side] = {} if ids is None else {str(int(i)):p.reshape(4,2).tolist() for i,p in zip(ids.ravel(),corners)}
            expected = {'left':24,'right':30}[waypoint['target']]
            row.update(planned_id=waypoint['planned_id'], observer=waypoint['observer'], target=waypoint['target'],
                       camera_images=camera_images, camera_detections=camera_detections,
                       frame_sequence=waypoint['frame_sequence'], timestamp_s=waypoint['timestamp_s'],
                       synchronized_render_step=True, expected_marker_id=expected,
                       valid_for_fit=True, marker_detected=str(expected) in camera_detections[waypoint['observer']])
        (output/(name+'.json')).write_text(json.dumps(row))
        return row


def serve(session, token, port):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def log_message(self, *_):
            pass

        def send(self, status, data, kind='application/json', metadata=None):
            if not isinstance(data, bytes):
                data = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            if metadata is not None:
                self.send_header('X-Rendered-Frame', json.dumps(metadata, allow_nan=False, separators=(',', ':')))
            self.end_headers()
            self.wfile.write(data)

        def authorized(self):
            host = self.headers.get('Host', '')
            origin = self.headers.get('Origin')
            actual_port = self.server.server_port
            allowed = host in (f'127.0.0.1:{actual_port}', f'localhost:{actual_port}')
            allowed = allowed and (origin is None or origin in (f'http://127.0.0.1:{actual_port}', f'http://localhost:{actual_port}'))
            return allowed and hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+token)

        def do_GET(self):
            if not self.authorized():
                return self.send(403, {'error': 'forbidden'})
            url = urlsplit(self.path)
            if url.path == '/stream.mjpg':
                return self.stream(parse_qs(url.query).get('view', ['actual'])[0])
            response = (404, {'error': 'not found'})
            with session.lock:
                if url.path == '/state':
                    response = (200, session.state())
                elif url.path == '/frame.jpg':
                    view = parse_qs(url.query).get('view', ['actual'])[0]
                    image = session.jpeg.get(view)
                    frame_id = parse_qs(url.query).get('frame_id', [None])[0]
                    candidate_id = parse_qs(url.query).get('candidate_id', [None])[0]
                    wrist_side = view.split('-')[0] if view in ('left-calibration','left-calibration-original','right-calibration','right-calibration-original') else None
                    display_meta=session.frame_metadata.get(view) if view in ('actual','preview') else None
                    requested={key:parse_qs(url.query).get(key,[None])[0] for key in ('renderer_instance_id','target_revision','overview_revision')}
                    if view in ('actual','preview') and any(value is not None and (not display_meta or str(display_meta.get(key))!=value) for key,value in requested.items()):
                        response=(409,{'error':'display frame revision differs; wait for current renderer frame'})
                    elif wrist_side and candidate_id is not None and candidate_id != session.wrist_overlay_state.get(wrist_side,{}).get('id'):
                        response = (409, {'error': 'wrist calibration changed; refresh state'})
                    elif view in ('calibration', 'calibration-original') and frame_id is not None and frame_id != session.calibration_state.get('calibration_frame_id'):
                        response = (409, {'error': 'calibration frame advanced; refresh state'})
                    elif image:
                        response = (200, image, 'image/jpeg')
                        if display_meta is not None:response+=(copy.deepcopy(display_meta),)
                        elif view in ('head', 'left', 'right'):
                            camera = (session.camera if view == 'head' or session.wrist_contract is None
                                      else session.wrist_contract['cameras'][view])
                            response += ({'source': 'dsw_rtx_kinematic_simulation', 'view': view,
                                          'frame_sequence': session.frame_sequence,
                                          'timestamp_s': session.frame_timestamp.get(view, session.frame_timestamp.get('head', time.time())),
                                          'q_reported_rad': session.actual_q.tolist(),
                                          'jaw_gap_mm': session.actual_gaps.tolist(), 'camera': camera},)
                elif url.path == '/dataset' and session.dataset is not None:
                    response = (200, session.dataset)
                elif url.path.startswith('/artifacts/') and session.dataset is not None:
                    name = url.path[len('/artifacts/'):]
                    if re.fullmatch(r'(frame-\d{3}(?:-(?:left|right))?\.(png|json)|camera\.json|robot\.urdf|dataset\.json|completion\.json)', name):
                        path = session.batch_dir/name
                        if path.is_file():
                            response = (200, path.read_bytes(), 'image/png' if name.endswith('.png') else 'application/octet-stream')
            self.send(*response)

        def stream(self, view):
            if view not in ('actual', 'preview', 'head'):
                return self.send(400, {'error': 'invalid stream view'})
            with session.lock:
                ready = session.jpeg.get(view) is not None
            if not ready:
                return self.send(503, {'error': 'frame not ready'})
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last, sent = None, 0.
            try:
                while True:
                    with session.frame_ready:
                        session.frame_ready.wait_for(lambda: session.frame_timestamp.get(view) != last, timeout=1.)
                        stamp, frame = session.frame_timestamp.get(view), session.jpeg.get(view)
                        metadata=copy.deepcopy(session.frame_metadata.get(view))
                    if frame is None:
                        continue
                    # Send idle previews as heartbeats so disconnected viewers are
                    # detected by write(), rather than leaking an SSH channel forever.
                    time.sleep(max(0., 1/30-(time.monotonic()-sent)))
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+
                                     b'\r\nX-Frame-Timestamp: '+str(stamp).encode()+
                                     (b'\r\nX-Rendered-Frame: '+json.dumps(metadata,allow_nan=False,separators=(',',':')).encode() if metadata else b'')+
                                     b'\r\n\r\n'+frame+b'\r\n')
                    self.wfile.flush()
                    last, sent = stamp, time.monotonic()
            except (OSError, TimeoutError):
                return

        def do_POST(self):
            if not self.authorized():
                return self.send(403, {'error': 'forbidden'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= (2000000 if self.path == '/real-overlay' else 200000):
                    raise ValueError('invalid body size')
                body = json.loads(self.rfile.read(length))
                if self.path == '/real-overlay':
                    return self.send(200, real_overlay_request(session.geometry, body), 'image/jpeg')
                self.send(200, session.command(self.path, body))
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                self.send(400, {'error': str(exc)})

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def real_overlay_request(geometry, body):
    """Stateless real-camera projection on DSW; never reads simulation pixels/q."""
    import base64
    import cv2
    candidate = body['candidate']
    validate_calibration(candidate)
    camera = body['camera']
    values = np.asarray([camera.get(k) for k in ('fx','fy','cx','cy')],float)
    if (any(type(camera.get(k)) is not int or not 1 <= camera[k] <= maximum for k,maximum in (('width',1920),('height',1080))) or
            not np.isfinite(values).all() or np.any(values[:2] <= 0)):
        raise ValueError('invalid real camera dimensions or intrinsics')
    image = cv2.imdecode(np.frombuffer(base64.b64decode(body['jpeg_base64'], validate=True), np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (camera['height'],camera['width']):
        raise ValueError('real camera JPEG dimensions differ from intrinsics')
    from arx_env.joint_head_gpu_overlay import GPUOverlay
    if candidate.get('T_link6_marker') is not None:
        # One selected frozen support mesh; never mutate the head renderer/cache.
        key = contract_sha256(candidate['T_link6_marker'])
        lock = geometry.__dict__.setdefault('_wrist_gpu_cache_lock', threading.Lock())
        with lock:
            cache = getattr(geometry,'_wrist_gpu_overlay_cache',None)
            if cache is None or cache[0] != key:
                view = marker_geometry(geometry,candidate)
                cache = (key, GPUOverlay(view))
                geometry._wrist_gpu_overlay_cache = cache
            overlay = cache[1]
    else:
        if not hasattr(geometry, '_real_gpu_overlay'):
            geometry._real_gpu_overlay = GPUOverlay(geometry)
        overlay = geometry._real_gpu_overlay
    rendered = overlay.render(image, body['qpos'], candidate, camera)
    ok, encoded = cv2.imencode('.jpg', rendered, [cv2.IMWRITE_JPEG_QUALITY,85])
    if not ok:raise ValueError('real overlay encoding failed')
    return encoded.tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8118)
    parser.add_argument('--render-gpu', type=int, default=0,
                        help='Physical Vulkan GPU index; configure CUDA_VISIBLE_DEVICES separately for CUDA overlays')
    parser.add_argument('--duration-s', type=float, default=0,
                        help='Optional runtime limit in seconds; 0 keeps the service running')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--display-only',action='store_true')
    parser.add_argument('--execution-json')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not 0 <= args.duration_s <= 14400 or args.render_gpu < 0:
        raise ValueError('invalid port/lifetime/render GPU')
    token = args.token_file.read_text().strip()
    if len(token) < 32:
        raise ValueError('token must have at least 32 characters')
    from arx_env.joint_head_workbench_geometry import Geometry
    config = json.loads(args.config.read_text())
    if args.display_only:config['display_only']=True
    if config.get('jaw_mapping_file'):
        from arx_env.joint_head_jaw_mapping import configure_mapping
        configure_mapping(config['jaw_mapping_file'])
    geometry = Geometry(config)
    if args.dry_run:
        q = geometry._q(config.get('initial_q', [0.]*12))
        if not geometry.preview(q, False)['in_limits']:
            raise ValueError('initial pose outside joint limits')
        print(json.dumps({'valid': True, 'mode': 'simulation', 'hardware': False}))
        return
    if args.output.exists():
        raise FileExistsError(args.output)
    session = Session(geometry, args.output, config)
    registration=None
    if args.execution_json:
        from local_execution import register_role
        registration=register_role(json.loads(args.execution_json),'preview',__file__,instance_id=session.renderer_instance_id)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'active_gpu': args.render_gpu, 'multi_gpu': False, 'renderer': 'RayTracedLighting',
                         'limit_cpu_threads':len(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else 4})
    server = None
    try:
        renderer = RTXRenderer(geometry)
        session.tick(renderer, 0)
        server = serve(session, token, args.port)
        if registration:registration('ready')
        deadline = time.monotonic()+args.duration_s if args.duration_s else float('inf')
        previous = time.monotonic()
        while time.monotonic() < deadline and app.is_running():
            now = time.monotonic()
            rendered = session.tick(renderer, now-previous)
            previous = now
            if not rendered:
                app.update()
            time.sleep(max(0., 1/30-(time.monotonic()-now)))
        session.command('/stop', {})
        session.tick(renderer, 0)
    finally:
        if server:
            server.shutdown()
        app.close()
        if registration:registration('stopped')


if __name__ == '__main__':
    main()
