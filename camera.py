"""Opt-in local camera capture. No device opens during discovery."""
from __future__ import annotations

from dataclasses import dataclass, field
import threading
import os
import time
from pathlib import Path
from typing import Callable


@dataclass
class _Stream:
    capture: object
    stop: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None
    jpeg: bytes | None = None
    sequence: int = 0
    last_frame: float = 0.0
    error: str | None = None
    estimation: bool = False
    model_path: str = ''
    generation: int = 0
    estimate: dict | None = None
    estimate_error: str | None = None
    estimate_sequence: int = 0
    next_estimate: float = 0.0


def enumerate_devices(root: Path = Path('/sys/class/video4linux')) -> list[dict]:
    """List Linux video nodes using metadata only; never opens a camera."""
    devices = []
    for node in sorted(root.glob('video*')):
        name_file = node / 'name'
        if name_file.exists():
            try:
                name = name_file.read_text(errors='replace').strip() or node.name
            except OSError:
                name = node.name
            devices.append({'id': f'/dev/{node.name}', 'name': name})
    return devices


class CameraManager:
    """Bounded latest-JPEG capture sessions, enabled one device at a time or many."""

    def __init__(self, devices: Callable[[], list[dict]] = enumerate_devices,
                 capture_factory: Callable[[str], object] | None = None,
                 jpeg_encoder: Callable[[object], bytes | None] | None = None,
                 estimator_factory: Callable[[str], object] | None = None,
                 max_jpeg_bytes: int = 2_000_000):
        self._devices = devices
        self._capture_factory = capture_factory
        self._jpeg_encoder = jpeg_encoder
        self._estimator_factory = estimator_factory
        self._max_jpeg_bytes = max_jpeg_bytes
        self._streams: dict[str, _Stream] = {}
        self._lock = threading.Lock()
        self._ops = threading.Lock()

    def _encode(self, frame) -> bytes | None:
        if self._jpeg_encoder:
            encoded = self._jpeg_encoder(frame)
            return encoded if encoded and len(encoded) <= self._max_jpeg_bytes else None
        try:
            import cv2
        except ImportError:
            raise RuntimeError('OpenCV is required for live camera capture (pip install opencv-python)')
        ok, value = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return None
        encoded = value.tobytes()
        return encoded if len(encoded) <= self._max_jpeg_bytes else None

    def _factory(self, device_id: str):
        if self._capture_factory:
            return self._capture_factory(device_id)
        try:
            import cv2
        except ImportError:
            raise RuntimeError('OpenCV is required for live camera capture (pip install opencv-python)')
        return cv2.VideoCapture(device_id)

    def _overlay(self, frame, result):
        try:
            import cv2
            import numpy as np
            if not isinstance(frame, np.ndarray) or frame.ndim != 3:
                return frame
            points = {p.get('name'): p for p in result.landmarks if p.get('visibility', 1) >= .5}
            links = (('left_shoulder','right_shoulder'), ('left_shoulder','left_elbow'),
                     ('left_elbow','left_wrist'), ('right_shoulder','right_elbow'),
                     ('right_elbow','right_wrist'), ('left_shoulder','left_hip'),
                     ('right_shoulder','right_hip'), ('left_hip','right_hip'),
                     ('left_hip','left_knee'), ('left_knee','left_ankle'),
                     ('right_hip','right_knee'), ('right_knee','right_ankle'))
            out = frame.copy(); height, width = out.shape[:2]
            xy = {name: (int(point['x'] * width), int(point['y'] * height)) for name, point in points.items()}
            for first, second in links:
                if first in xy and second in xy:
                    cv2.line(out, xy[first], xy[second], (90, 235, 218), 1, cv2.LINE_AA)
            for point in xy.values():
                cv2.circle(out, point, 3, (183, 160, 255), -1, cv2.LINE_AA)
            return out
        except Exception:
            return frame

    @staticmethod
    def _describe(item, stream):
        live = bool(stream and not stream.stop.is_set() and not stream.error)
        stale = bool(stream and stream.sequence and time.monotonic() - stream.last_frame > 2)
        return {**item, 'streaming': live,
                'frame_stale': stale,
                'sequence': stream.sequence if stream else 0,
                'error': stream.error if stream else None,
                'estimation': bool(live and stream.estimation),
                'estimate': stream.estimate if live and not stale else None,
                'estimate_sequence': stream.estimate_sequence if live else 0,
                'estimate_error': stream.estimate_error if stream else None}

    def devices(self) -> list[dict]:
        items = self._devices()
        known = {item['id'] for item in items}
        with self._lock:
            items += [{'id': key, 'name': key + ' (disconnected)'}
                      for key in self._streams if key not in known]
            return [self._describe(item, self._streams.get(item['id'])) for item in items]

    def _run(self, device_id: str, stream: _Stream) -> None:
        estimator = None  # Only this thread may construct, use, or close the model.
        estimator_generation = -1
        try:
            while not stream.stop.is_set():
                ok, frame = stream.capture.read()
                captured_at = time.monotonic()  # Arrival time, not sensor exposure time.
                if not ok:
                    stream.error = 'Camera returned no frame'
                    stream.stop.set()
                    break
                shape = getattr(frame, 'shape', ())
                if len(shape) == 3 and (shape[1] > 640 or shape[0] > 480):
                    import cv2
                    scale = min(640 / shape[1], 480 / shape[0])
                    frame = cv2.resize(frame, (round(shape[1]*scale), round(shape[0]*scale)))
                with self._lock:
                    enabled, generation, model_path = stream.estimation, stream.generation, stream.model_path
                if estimator_generation != generation:
                    if estimator is not None:
                        estimator.close()
                    estimator = None
                    estimator_generation = generation
                if enabled and captured_at < stream.next_estimate:
                    continue  # Consume newest frames without publishing unmatched overlays.
                result = None
                estimate_error = None
                if enabled:
                    stream.next_estimate = captured_at + .1
                    try:
                        if estimator is None:
                            if self._estimator_factory:
                                estimator = self._estimator_factory(model_path)
                            else:
                                from estimator import PoseEstimator
                                estimator = PoseEstimator(model_path)
                        result = estimator.estimate(frame)
                    except Exception as exc:
                        estimate_error = str(exc)
                output_frame = self._overlay(frame, result) if result is not None else frame
                encoded = self._encode(output_frame)
                if encoded is not None:
                    with self._lock:
                        if stream.stop.is_set() or generation != stream.generation:
                            continue
                        stream.jpeg = encoded
                        stream.sequence += 1
                        stream.last_frame = captured_at
                        if enabled:
                            stream.estimate = None if result is None else {
                                'landmarks': list(result.landmarks)[:33],
                                'inferred3d': [list(point) for point in result.inferred3d[:33]],
                                'coordinate_note': result.coordinate_note,
                                'timestamp': captured_at,
                                'processing_ms': (time.monotonic() - captured_at) * 1000,
                            }
                            stream.estimate_error = estimate_error
                            stream.estimate_sequence = stream.sequence
                            if estimate_error:
                                stream.estimation = False
                                stream.generation += 1
        except Exception as exc:  # device/backend errors must not kill HTTP server
            stream.error = str(exc)
        finally:
            stream.stop.set()
            if estimator is not None:
                try:
                    estimator.close()
                except Exception:
                    pass
            try:
                stream.capture.release()
            except Exception:
                pass

    def set_enabled(self, device_id: str, enabled: bool) -> dict:
        known = {d['id'] for d in self._devices()}
        with self._lock:
            existing = device_id in self._streams
        if device_id not in known and (enabled or not existing):
            raise KeyError(device_id)
        if not enabled:
            self.stop(device_id)
            return self.status(device_id)
        with self._ops:
            with self._lock:
                existing = self._streams.get(device_id)
                if existing and existing.thread and existing.thread.is_alive() and not existing.stop.is_set():
                    return self._status_locked(device_id)
                if existing:
                    existing.stop.set()
                    old_thread = existing.thread
                else:
                    old_thread = None
            if old_thread and old_thread.is_alive():
                old_thread.join(timeout=1)
                if old_thread.is_alive():
                    raise RuntimeError('Camera is still stopping')
            with self._lock:
                if self._streams.get(device_id) is existing:
                    self._streams.pop(device_id, None)
            capture = self._factory(device_id)
            if hasattr(capture, 'isOpened') and not capture.isOpened():
                try:
                    capture.release()
                except Exception:
                    pass
                raise RuntimeError('Camera could not be opened')
            for prop, value in ((3, 640), (4, 480), (5, 15)):
                try:
                    if hasattr(capture, 'set'):
                        capture.set(prop, value)
                except Exception:
                    pass
            stream = _Stream(capture)
            stream.thread = threading.Thread(target=self._run, args=(device_id, stream), daemon=True)
            with self._lock:
                self._streams[device_id] = stream
                stream.thread.start()
                return self._status_locked(device_id)

    def stop(self, device_id: str) -> None:
        with self._ops:
            with self._lock:
                stream = self._streams.get(device_id)
                if stream:
                    stream.stop.set()
            if stream and stream.thread and stream.thread is not threading.current_thread():
                stream.thread.join(timeout=1)
            with self._lock:
                if self._streams.get(device_id) is stream:
                    if not stream or not stream.thread or not stream.thread.is_alive():
                        self._streams.pop(device_id, None)
                    else:
                        stream.error = 'Camera is stopping'

    def _status_locked(self, device_id: str) -> dict:
        item = next((item for item in self._devices() if item['id'] == device_id),
                    {'id': device_id, 'name': device_id + ' (disconnected)'})
        return self._describe(item, self._streams.get(device_id))

    def status(self, device_id: str) -> dict:
        with self._lock:
            return self._status_locked(device_id)

    def frame(self, device_id: str) -> bytes | None:
        with self._lock:
            stream = self._streams.get(device_id)
            return stream.jpeg if (stream and not stream.stop.is_set() and not stream.error
                                   and time.monotonic() - stream.last_frame <= 2) else None

    def set_estimation(self, device_id: str, enabled: bool) -> dict:
        with self._lock:
            stream = self._streams.get(device_id)
            if not stream or stream.stop.is_set() or stream.error:
                raise RuntimeError('Start camera before enabling estimation')
            if enabled == stream.estimation:
                return self._status_locked(device_id)
            model_path = os.environ.get('NX_FUSE_MODEL')
            if enabled and not model_path:
                raise RuntimeError('NX_FUSE_MODEL must name a pose model before enabling estimation')
            if enabled and not self._estimator_factory and not Path(model_path).is_file():
                raise RuntimeError('NX_FUSE_MODEL does not name an existing model file')
            stream.model_path = model_path or ''
            stream.estimation = enabled
            stream.generation += 1
            stream.estimate = stream.jpeg = None
            stream.estimate_error = None
            stream.estimate_sequence = 0
            stream.next_estimate = 0
            return self._status_locked(device_id)

    def close(self) -> None:
        for device_id in list(self._streams):
            self.stop(device_id)
