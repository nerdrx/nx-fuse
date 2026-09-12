"""Explicit stationary-anchor camera alignment; observation-only, no VR output."""
import hashlib
import json
import math
import secrets
import threading
import time


class AlignmentSetup:
    def __init__(self, cameras, lens, tracking):
        self.cameras, self.lens, self.tracking = cameras, lens, tracking
        self._lock = threading.RLock()
        self._reset()

    def _reset(self):
        self._context = None
        self._samples = []
        self._pending = None
        self._profile = None
        self._error = ''

    def _current(self, camera):
        if not self.tracking:
            raise ValueError('Launch the tracking lab and connect WiVRn first')
        feed = self.tracking.snapshot()
        if not feed['connected']:
            raise ValueError('Waiting for fresh WiVRn tracking')
        lens = self.lens.snapshot()['profile']
        if not lens or not lens.get('validated') or lens.get('camera_id') != camera:
            raise ValueError('Solve a validated lens profile for this camera first')
        status = self.cameras.status(camera)
        if not status['streaming'] or status['frame_stale']:
            raise ValueError('Camera is stopped or stale')
        if list(status['image_size']) != list(lens['image_size']):
            raise ValueError('Camera resolution no longer matches the lens profile')
        signature = hashlib.sha256(json.dumps(lens, sort_keys=True).encode()).hexdigest()
        return (camera, feed['generation'], status['epoch'], signature), lens

    def _check_context(self):
        if self._context is None:
            return
        try:
            current, _ = self._current(self._context[0])
            if current != self._context:
                raise ValueError('Camera, lens profile or VR space changed')
        except ValueError as exc:
            self._reset()
            self._error = str(exc) + '; collect a new alignment'

    def snapshot(self):
        with self._lock:
            self._check_context()
            if self._pending and time.monotonic() > self._pending['expires']:
                self._pending = None
                self._error = 'Frozen frame expired; freeze another view'
            pending = self._pending
            return {'count': len(self._samples), 'error': self._error,
                    'camera': self._context[0] if self._context else '',
                    'generation': self._context[1] if self._context else '',
                    'profile': self._profile, 'pending': None if pending is None else {
                        key: pending[key] for key in ('token', 'width', 'height', 'anchor')},
                    'read_only': True}

    def frame(self, token):
        with self._lock:
            self.snapshot()
            return self._pending['jpeg'] if self._pending and token == self._pending['token'] else None

    def command(self, value):
        with self._lock:
            try:
                self._command(value)
                self._error = ''
            except (ValueError, RuntimeError, KeyError) as exc:
                self._error = str(exc)
                raise

    def _command(self, value):
        action = value.get('action')
        if action == 'reset':
            self._reset()
            return
        if action not in ('freeze', 'sample', 'solve'):
            raise ValueError('Expected freeze, sample, solve or reset')
        self._check_context()
        camera = value.get('id')
        if not isinstance(camera, str) or not camera:
            raise ValueError('Select a camera')
        current, lens = self._current(camera)
        if self._context and self._context != current:
            raise ValueError('Reset before switching cameras')
        if action == 'freeze':
            if len(self._samples) >= 60:
                raise ValueError('Sixty samples collected; solve or reset')
            anchor = value.get('anchor', 'head')
            offset = value.get('offset', [0, 0, 0])
            frame = self.cameras.frame_snapshot(camera)
            reference = self.tracking.stable_anchor(anchor, frame['time_ns'], offset)
            if reference['generation'] != current[1] or frame['epoch'] != current[2]:
                raise ValueError('Tracking or camera changed during capture; try again')
            if list(frame['image_size']) != list(lens['image_size']):
                raise ValueError('Frame resolution differs from calibrated resolution')
            self._context = current
            self._pending = {'token': secrets.token_hex(12), 'jpeg': frame['jpeg'],
                             'width': frame['image_size'][0], 'height': frame['image_size'][1],
                             'anchor': anchor, 'offset': list(offset), 'position': reference['position'],
                             'match_error_ms': reference['match_error_ms'],
                             'expires': time.monotonic() + 60}
            return
        if action == 'sample':
            pending = self._pending
            if (not pending or value.get('token') != pending['token']
                    or time.monotonic() > pending['expires']):
                raise ValueError('Freeze a fresh view before selecting its reference point')
            pixel = value.get('pixel')
            if (not isinstance(pixel, list) or len(pixel) != 2
                    or any(type(v) not in (float, int) or not math.isfinite(v) for v in pixel)
                    or not 0 <= pixel[0] < pending['width'] or not 0 <= pixel[1] < pending['height']):
                raise ValueError('Select a point inside the frozen camera image')
            if any(math.dist(old['position'], pending['position']) < .03 for old in self._samples):
                raise ValueError('Move the reference point at least 3 cm; vary width, height and depth')
            self._samples.append({key: pending[key] for key in ('anchor', 'offset', 'position', 'match_error_ms')} | {'pixel': pixel})
            self._pending = self._profile = None
            return
        if len(self._samples) < 12:
            raise ValueError('Collect at least twelve varied reference positions')
        from spatial_alignment import solve_alignment
        report = solve_alignment([s['position'] for s in self._samples],
                                 [s['pixel'] for s in self._samples], lens['camera_matrix'],
                                 lens['distortion'], lens['image_size'])
        self._profile = {**report, 'camera_id': camera, 'image_size': lens['image_size'],
                         'generation': current[1], 'camera_epoch': current[2],
                         'lens_signature': current[3], 'samples': self._samples.copy(),
                         'scope': 'camera extrinsics only; inferred body depth remains uncalibrated',
                         'timing': 'stationary anchor matched to frame arrival, not measured exposure',
                         'read_only': True}
