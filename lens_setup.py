"""In-memory, explicit chessboard collection for the native setup page."""
from dataclasses import asdict
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
