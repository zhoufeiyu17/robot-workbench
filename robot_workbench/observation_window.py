"""Shared live/detector/bridge rule for a continuous, stable observation window."""
import json
import math

WINDOW_NS = 3_000_000_000
MINIMUM_FRAMES = 9
CUP_SPAN_M = .006


class ContinuousObservationWindow:
    def __init__(self):
        self.frames = []
        self.events = []
        self.identity = None
        self.previous_stamp = 0
        self.pair = None

    def add(self, observation):
        stamp = observation.get('source_stamp_ns')
        if type(stamp) is not int or stamp <= self.previous_stamp:
            raise ValueError('observation source time must strictly increase')
        previous_stamp = self.previous_stamp
        self.previous_stamp = stamp
        identity = tuple(observation.get(k) for k in ('tracking_session_id', 'runtime_config_id', 'frame_id'))
        if any(not isinstance(v, str) or not v for v in identity):
            raise ValueError('observation runtime/session/frame identity required')
        if self.identity is not None and identity != self.identity:
            raise ValueError('observation runtime/session/frame identity changed')
        self.identity = identity
        if observation.get('provider_status') not in ('OK', 'SKIPPED'):
            raise ValueError('perception provider failed')
        valid = observation.get('pair_valid')
        if type(valid) is not bool:
            raise ValueError('pair_valid must be boolean')
        reason = 'detection_missing' if not valid else None
        if previous_stamp and stamp - previous_stamp > 1_000_000_000:
            reason = 'capture_gap'
        distance = None
        if valid:
            tracks = observation.get('tracks')
            if (not isinstance(tracks, list) or len(tracks) != 2 or
                    any(not isinstance(t, dict) for t in tracks) or
                    sorted(t.get('object_type', '') for t in tracks) != ['coffee_machine', 'cup'] or
                    any(t.get('pose_current') is not True for t in tracks)):
                raise ValueError('valid detection requires two current object tracks')
            pair = tuple(sorted((t['object_type'], str(t['track_id']),
                json.dumps(t['provenance']['asset'], sort_keys=True)) for t in tracks))
            cup = next(t for t in tracks if t['object_type'] == 'cup')['world']['xyz_m']
            if len(cup) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in cup):
                raise ValueError('finite world cup position required')
            if self.pair is not None and pair != self.pair:
                reason = 'track_or_asset_changed'
            elif self.frames:
                points = [next(t for t in f['tracks'] if t['object_type'] == 'cup')['world']['xyz_m']
                          for f in self.frames]
                distance = max(math.dist(cup, point) for point in points)
                # Include the exact boundary despite floating-point subtraction.
                if distance >= CUP_SPAN_M or math.isclose(distance, CUP_SPAN_M, rel_tol=0, abs_tol=1e-12):
                    reason = 'cup_position_span'
            self.pair = pair
        if reason:
            self.events.append(dict(source_stamp_ns=stamp, reason=reason, displacement_m=distance,
                threshold_m=CUP_SPAN_M, discarded_frames=len(self.frames), boundary_frame_excluded=True))
            self.frames.clear()
            self.pair = None
            # The anomalous sample is never the first sample of the new window.
            return self.status()
        self.frames.append(observation)
        return self.status()

    def status(self):
        span = (self.frames[-1]['source_stamp_ns'] - self.frames[0]['source_stamp_ns']) if self.frames else 0
        return dict(ready=span >= WINDOW_NS and len(self.frames) >= MINIMUM_FRAMES,
                    span_s=span/1e9, frame_count=len(self.frames), required_span_s=WINDOW_NS/1e9,
                    minimum_frames=MINIMUM_FRAMES, reobserve_count=len(self.events),
                    last_reset=self.events[-1] if self.events else None,
                    valid_stamps_ns=[f['source_stamp_ns'] for f in self.frames])
