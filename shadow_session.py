"""Bounded, opt-in camera body observations, deliberately separate from VR output."""
import math
import threading
import time


class ShadowSession:
    def __init__(self, cameras, lens, alignment, tracking):
        self.cameras, self.lens, self.alignment, self.tracking = cameras, lens, alignment, tracking
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._enabled = False
        self._camera = ''
        self._version = 0
        self._result = None
        self._recording = False
        self._frames = []
        self._reason = 'Enable the anchored body preview after camera alignment'

    def _context(self, camera):
        if not self.tracking:
            raise ValueError('Launch the tracking lab and connect a headset')
        profile = self.alignment.snapshot()['profile']
        if not profile or not profile.get('validated') or profile['camera_id'] != camera:
            raise ValueError('Complete camera-to-VR alignment for this camera first')
        lens = self.lens.snapshot()['profile']
        if not lens or not lens.get('validated') or lens['camera_id'] != camera:
            raise ValueError('A matching validated lens profile is required')
        status = self.cameras.status(camera)
        if not status['streaming'] or not status.get('estimation') or status.get('frame_stale'):
            raise ValueError('Start this camera and enable body estimation')
        if status['epoch'] != profile['camera_epoch'] or list(status['image_size']) != list(profile['image_size']):
            raise ValueError('Camera changed; align it again')
        # Compare the actual solved transform too: a re-solve can share lens and VR generations.
        stamp = repr((profile['generation'], profile['camera_epoch'], profile['lens_signature'], profile['camera_to_vr']))
        return profile, lens, stamp

    def set_enabled(self, camera, enabled):
        if type(enabled) is not bool or not isinstance(camera, str) or not camera:
            raise ValueError('Expected a camera id and boolean enabled')
        if enabled:
            self._context(camera)
        with self._lock:
            self._version += 1
            self._enabled, self._camera = enabled, camera
            self._result = None
            if not enabled: self._recording = False
            self._reason = 'Waiting for a matched camera/headset observation' if enabled else 'Preview is off'
            if enabled and self._thread is None:
                self._thread = threading.Thread(target=self._run, daemon=True)
                self._thread.start()

    def _run(self):
        while not self._stop.wait(.1):
            with self._lock:
                enabled, camera, version = self._enabled, self._camera, self._version
            if not enabled:
                continue
            try:
                profile, lens, stamp = self._context(camera)
                frame = self.cameras.estimate_snapshot(camera)
                if frame['epoch'] != profile['camera_epoch'] or list(frame['image_size']) != list(profile['image_size']):
                    raise ValueError('Camera changed during body estimation')
                key = (stamp, frame['sequence'])
                with self._lock:
                    previous = self._result
                if previous and previous['_key'] == key:
                    continue
                tracking = self.tracking.history_snapshot(frame['time_ns'])
                if tracking['generation'] != profile['generation']:
                    raise ValueError('VR space changed; align this camera again')
                records = {r['joint']:r for r in tracking['records']}
                from shadow_pose import reconstruct
                body = reconstruct(frame['estimate'], lens, profile, records['head'])
                joints = []
                for name, candidate in body['joints'].items():
                    baseline = records.get(name, {}).get('position')
                    joints.append({'joint':name, **candidate, 'baseline':baseline,
                                   'residual_m':math.dist(candidate['position'], baseline) if baseline is not None else None})
                # Computation is outside all capture and transport locks. Recheck before publishing.
                if self._context(camera)[2] != stamp:
                    raise ValueError('Alignment changed during reconstruction')
                result = {'joints':joints, 'metrics':{'rms_px':body['rms_px'],
                          'anchor_shift_m':body['anchor_shift_m'], 'match_error_ms':tracking['match_error_ms']},
                          'note':body['note'], '_time_ns':frame['time_ns'], '_key':key, '_stamp':stamp}
                with self._lock:
                    if version == self._version:
                        self._result, self._reason = result, ''
                        if self._recording:
                            self._frames.append({'camera':camera, 'generation':profile['generation'],
                                                 'timestamp_ns':frame['time_ns'], 'joints':joints,
                                                 'metrics':result['metrics'], 'note':body['note']})
                            if len(self._frames) >= 600: self._recording = False
            except Exception as exc:
                with self._lock:
                    if version == self._version:
                        self._result, self._reason = None, str(exc)

    def snapshot(self):
        with self._lock:
            enabled, camera, result, reason = self._enabled, self._camera, self._result, self._reason
            recording, count = self._recording, len(self._frames)
        if enabled:
            try:
                stamp = self._context(camera)[2]
                if result and stamp != result['_stamp']:
                    result, reason = None, 'Alignment changed; waiting for a new observation'
            except ValueError as exc:
                result, reason = None, str(exc)
        age = (time.monotonic_ns()-result['_time_ns'])/1e6 if result else 0
        if result and not 0 <= age <= 500:
            result, reason = None, 'Camera observation expired'
        return {'recording':recording, 'recorded_frames':count, 'recording_full':count >= 600,
                'enabled':enabled, 'mode':'shadow', 'camera':camera, 'available':bool(result),
                'reason':reason, 'joints':result['joints'] if result else [],
                'metrics':{**result['metrics'], 'age_ms':age} if result else {},
                'note':result['note'] if result else 'Read-only observation. No poses are sent to VR.'}

    def recording_command(self, action):
        with self._lock:
            if action == 'clear':
                self._recording = False
                self._frames.clear()
            elif action == 'stop':
                self._recording = False
            elif action == 'start':
                if not self._enabled:
                    raise ValueError('Enable the body preview first')
                if len(self._frames) >= 600:
                    raise ValueError('Pose log is full; export and clear it before starting again')
                self._recording = True
            else:
                raise ValueError('Expected start, stop or clear')

    def recording(self):
        with self._lock:
            return {'format':'nx-fuse-shadow-v1', 'note':'Model-inferred observations, not measured tracking accuracy. No images.',
                    'frames':self._frames.copy()}

    def close(self):
        with self._lock:
            self._enabled = False
            self._version += 1
            self._result = None
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
