"""In-memory, explicit chessboard collection for the native setup page."""
from dataclasses import asdict
import json
import math
import threading


class LensSetup:
    def __init__(self):
        self._lock = threading.Lock()
        self._views = []
        self._camera = ''
        self._pattern = (9, 6)
        self._square = .025
        self._busy = False
        self._error = ''
        self._profile = None

    @staticmethod
    def _numbers(value, name, length=None):
        if not isinstance(value, (list, tuple)) or (length is not None and len(value) != length):
            raise ValueError(f'{name} has invalid shape')
        if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
            raise ValueError(f'{name} must contain finite numbers')
        return tuple(float(x) for x in value)

    def load_profile(self, value):
        """Validate and load exported lens geometry; ignore calibration metadata."""
        try:
            if len(json.dumps(value, separators=(',', ':'))) > 32768:
                raise ValueError('Lens profile exceeds 32 KiB')
        except (TypeError, ValueError) as exc:
            raise ValueError('Lens profile must be a JSON object') from exc
        if not isinstance(value, dict):
            raise ValueError('Lens profile must be a JSON object')
        required = {'camera_id', 'image_size', 'camera_matrix', 'distortion', 'rms_error',
                    'train_errors', 'held_out_errors', 'train_indices', 'held_out_indices'}
        known = required | {'held_out_rms_px', 'scope', 'validated'}
        if set(value) - known or not required <= set(value):
            raise ValueError('Lens profile contains unknown or missing fields')
        camera_id = value['camera_id']
        if not isinstance(camera_id, str) or not camera_id.strip() or len(camera_id) >= 256:
            raise ValueError('camera_id must be a non-empty string shorter than 256 characters')
        size = value['image_size']
        if (not isinstance(size, (list, tuple)) or len(size) != 2
                or any(type(x) is not int or not 1 <= x <= 8192 for x in size)):
            raise ValueError('image_size must contain two positive integers <= 8192')
        width, height = size
        matrix_values = value['camera_matrix']
        if (not isinstance(matrix_values, (list, tuple)) or len(matrix_values) != 3
                or any(not isinstance(row, (list, tuple)) or len(row) != 3 for row in matrix_values)):
            raise ValueError('camera_matrix must be 3 x 3')
        matrix = tuple(self._numbers(row, 'camera_matrix', 3) for row in matrix_values)
        if matrix[0][0] <= 0 or matrix[1][1] <= 0 or matrix[0][0] > width * 20 or matrix[1][1] > height * 20 \
                or matrix[0][1] != 0 or matrix[1][0] != 0 or matrix[2] != (0.0, 0.0, 1.0) or not (-width <= matrix[0][2] <= 2 * width) \
                or not (-height <= matrix[1][2] <= 2 * height):
            raise ValueError('camera_matrix has implausible calibration values')
        distortion = self._numbers(value['distortion'], 'distortion')
        if len(distortion) not in (4, 5, 8, 12, 14) or any(abs(x) > 1e6 for x in distortion):
            raise ValueError('distortion has invalid length or bounds')
        rms = self._numbers([value['rms_error']], 'rms_error')[0]
        if rms < 0 or rms > 100:
            raise ValueError('rms_error exceeds limit')
        train_errors = self._numbers(value['train_errors'], 'train_errors')
        held_errors = self._numbers(value['held_out_errors'], 'held_out_errors')
        train_indices, held_indices = value['train_indices'], value['held_out_indices']
        if not isinstance(train_indices, (list, tuple)) or not isinstance(held_indices, (list, tuple)):
            raise ValueError('view indices must be arrays')
        train_indices, held_indices = tuple(train_indices), tuple(held_indices)
        if (len(train_errors) < 4 or not held_errors or any(x < 0 or x > 100 for x in train_errors + held_errors)
                or len(train_errors) != len(train_indices)
                or len(held_errors) != len(held_indices)):
            raise ValueError('calibration errors or index lengths are invalid')
        if (any(type(x) is not int or x < 0 for x in train_indices + held_indices)
                or len(set(train_indices + held_indices)) != len(train_indices) + len(held_indices)
                or len(train_indices) + len(held_indices) < 6):
            raise ValueError('view indices must be unique, disjoint, and contain six views')
        profile = {
            'camera_id': camera_id, 'image_size': tuple(size), 'camera_matrix': matrix,
            'distortion': distortion, 'rms_error': rms, 'train_errors': train_errors,
            'held_out_errors': held_errors, 'train_indices': tuple(train_indices),
            'held_out_indices': tuple(held_indices),
            'held_out_rms_px': math.sqrt(sum(x * x for x in held_errors) / len(held_errors)),
            'validated': (math.sqrt(sum(x * x for x in held_errors) / len(held_errors)) <= 1
                          and max(held_errors) <= 2),
            'scope': 'lens intrinsics only; not camera-to-VR calibration',
        }
        with self._lock:
            if self._busy:
                raise ValueError('Calibration is busy')
            self._views.clear()
            self._camera = camera_id
            self._profile = profile
            self._error = ''

    def snapshot(self):
        with self._lock:
            return {'camera': self._camera, 'count': len(self._views), 'busy': self._busy,
                    'error': self._error, 'profile': self._profile,
                    'pattern': self._pattern, 'square_mm': self._square * 1000}

    def command(self, value, cameras):
        action = value.get('action')
        if action not in ('capture', 'solve', 'reset'):
            raise ValueError('Expected capture, solve, or reset')
        with self._lock:
            if self._busy:
                raise ValueError('Calibration is busy')
            if action == 'reset':
                self._views.clear()
                self._profile = None
                self._error = self._camera = ''
                return
            if action == 'solve':
                if value.get('id') != self._camera:
                    raise ValueError('Select the camera used to capture these views')
                if len(self._views) < 6:
                    raise ValueError('Capture at least six varied chessboard views')
                self._busy = True
                threading.Thread(target=self._solve, daemon=True).start()
                return
            camera = value.get('id')
            pattern = value.get('pattern', [9, 6])
            square = value.get('square_mm', 25)
            if (not isinstance(camera, str) or not isinstance(pattern, list) or len(pattern) != 2
                    or any(type(v) is not int or not 2 <= v <= 15 for v in pattern)
                    or type(square) not in (int, float) or not 5 <= square <= 100):
                raise ValueError('Choose a camera, 2–15 inner corners per side and 5–100 mm squares')
            status = cameras.status(camera)
            if not status['streaming'] or status.get('estimation'):
                raise ValueError('Start the camera and turn off body estimation before collecting lens views')
            if self._views and (camera != self._camera or tuple(pattern) != self._pattern or square/1000 != self._square):
                raise ValueError('Reset before changing the camera or board dimensions')
            if len(self._views) >= 30:
                raise ValueError('Thirty views collected; solve or reset')
            jpeg = cameras.frame(camera)
            if not jpeg:
                raise ValueError('No fresh camera frame')
            self._camera, self._pattern, self._square = camera, tuple(pattern), square/1000
            self._busy = True
            self._error = ''
            threading.Thread(target=self._capture, args=(jpeg,), daemon=True).start()

    def _capture(self, jpeg):
        try:
            import cv2
            import numpy as np
            from lens_calibration import detect_corners
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            view = detect_corners(frame, self._pattern, self._square)
            with self._lock:
                if self._views and view.image_size != self._views[0].image_size:
                    raise ValueError('Camera resolution changed; reset calibration')
                if any(np.sqrt(np.mean((view.image_points[0] - old.image_points[0])**2)) < 5 for old in self._views):
                    raise ValueError('Move or tilt the board more before capturing another view')
                self._views.append(view)
                self._profile = None
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
        finally:
            with self._lock:
                self._busy = False

    def _solve(self):
        try:
            from lens_calibration import ChessboardCorners, calibrate_lens
            views = self._views
            dataset = ChessboardCorners(self._pattern, self._square, views[0].image_size,
                                       tuple(v.object_points[0] for v in views),
                                       tuple(v.image_points[0] for v in views), tuple(range(len(views))))
            report = calibrate_lens(dataset)
            profile = {**asdict(report), 'camera_id': self._camera,
                       'held_out_rms_px': report.held_out_rms,
                       'validated': report.held_out_rms <= 1 and max(report.held_out_errors) <= 2,
                       'scope': 'lens intrinsics only; not camera-to-VR calibration'}
            with self._lock:
                self._profile, self._error = profile, ''
        except Exception as exc:
            with self._lock:
                self._profile, self._error = None, str(exc)
        finally:
            with self._lock:
                self._busy = False
