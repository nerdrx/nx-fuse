"""Offline chessboard camera-lens calibration.

This module calibrates camera intrinsics and lens distortion only. It does not
estimate a camera-to-VR or camera-to-world transform.
"""
import argparse
import glob
import json
import math
from dataclasses import dataclass
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:  # Keep the rest of nx-fuse importable without camera extras.
    cv2 = None
    np = None


@dataclass(frozen=True)
class ChessboardCorners:
    pattern_size: tuple[int, int]
    square_size: float
    image_size: tuple[int, int]
    object_points: tuple
    image_points: tuple
    frame_indices: tuple[int, ...]


@dataclass(frozen=True)
class LensCalibrationReport:
    camera_matrix: tuple
    distortion: tuple
    image_size: tuple[int, int]
    rms_error: float
    train_errors: tuple[float, ...]
    held_out_errors: tuple[float, ...]
    train_indices: tuple[int, ...]
    held_out_indices: tuple[int, ...]

    @property
    def held_out_rms(self):
        return float(np.sqrt(np.mean(np.square(self.held_out_errors)))) if self.held_out_errors else None

    def project(self, points, rvec, tvec):
        _require_deps()
        values = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        projected, _ = cv2.projectPoints(values, rvec, tvec, np.asarray(self.camera_matrix),
                                         np.asarray(self.distortion))
        return projected.reshape(-1, 2)


def _require_deps():
    if cv2 is None or np is None:
        raise RuntimeError('numpy and OpenCV are required (install requirements-camera.txt)')


def _pattern(pattern_size):
    if (not isinstance(pattern_size, (tuple, list)) or len(pattern_size) != 2
            or any(isinstance(v, bool) or not isinstance(v, int) or v < 2 for v in pattern_size)):
        raise ValueError('pattern_size must contain two integers >= 2')
    return int(pattern_size[0]), int(pattern_size[1])


def _square_size(square_size):
    if isinstance(square_size, bool) or not isinstance(square_size, (int, float)):
        raise ValueError('square_size must be a finite positive number in metres')
    _require_deps()
    if not np.isfinite(square_size) or square_size <= 0:
        raise ValueError('square_size must be a finite positive number in metres')
    return float(square_size)


def _frame(frame):
    image = np.asarray(frame)
    if image.ndim not in (2, 3) or image.shape[0] < 32 or image.shape[1] < 32:
        raise ValueError('frame must be an image array')
    return image


def _object_grid(pattern, square):
    cols, rows = pattern
    return (np.array([(col, row, 0) for row in range(rows) for col in range(cols)], dtype=np.float32) * square)


def detect_corners(frame, pattern_size, square_size):
    """Detect one chessboard frame for interactive capture workflows."""
    _require_deps()
    pattern = _pattern(pattern_size)
    square = _square_size(square_size)
    image = _frame(frame)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=0)
    if not found:
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4))
    if not found or corners is None or not np.isfinite(corners).all():
        raise ValueError('chessboard not found in frame')
    return ChessboardCorners(pattern, square, (int(image.shape[1]), int(image.shape[0])),
                             (_object_grid(pattern, square),),
                             (np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2),), (0,))


def collect_corners(frames, pattern_size, square_size, *, min_views=6):
    """Detect chessboard corners in frames; return accepted views and metadata.

    Frames may be grayscale or BGR arrays. All frames must have one image size.
    ``pattern_size`` is inner corners as ``(columns, rows)``.
    """
    _require_deps()
    pattern = _pattern(pattern_size)
    square = _square_size(square_size)
    if isinstance(min_views, bool) or not isinstance(min_views, int) or min_views < 6:
        raise ValueError('min_views must be a finite integer >= 6')
    template = _object_grid(pattern, square)
    objects, images, indices = [], [], []
    image_size = None
    for index, frame in enumerate(frames):
        image = _frame(frame)
        size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = size
        elif size != image_size:
            raise ValueError('all calibration frames must have identical dimensions')
        try:
            detected = detect_corners(image, pattern, square)
        except ValueError:
            continue
        if detected.image_size == image_size:
            objects.append(template.copy())
            images.append(detected.image_points[0])
            indices.append(index)
    if len(images) < min_views:
        raise ValueError(f'found {len(images)} usable chessboard views; need at least {min_views}')
    return ChessboardCorners(pattern, square, image_size, tuple(objects), tuple(images), tuple(indices))


def _check_diversity(points, image_size):
    centers = np.array([p.reshape(-1, 2).mean(axis=0) for p in points])
    spread = np.ptp(centers, axis=0) / np.array(image_size)
    scales = np.array([np.ptp(p.reshape(-1, 2), axis=0).prod() for p in points])
    if np.max(spread) < 0.04 or np.ptp(scales) / max(np.mean(scales), 1.0) < 0.03:
        raise ValueError('chessboard views are degenerate; capture varied position and distance/tilt')


def _validate_dataset(dataset):
    if (len(dataset.object_points) != len(dataset.image_points)
            or len(dataset.frame_indices) != len(dataset.image_points)):
        raise ValueError('chessboard dataset fields must have equal view counts')
    width, height = dataset.image_size
    if width <= 0 or height <= 0:
        raise ValueError('image_size must contain positive dimensions')
    expected = dataset.pattern_size[0] * dataset.pattern_size[1]
    for objects, images in zip(dataset.object_points, dataset.image_points):
        obj = np.asarray(objects, dtype=np.float32).reshape(-1, 3)
        img = np.asarray(images, dtype=np.float32).reshape(-1, 2)
        if obj.shape != (expected, 3) or img.shape != (expected, 2) or not np.isfinite(obj).all() or not np.isfinite(img).all():
            raise ValueError('chessboard points must contain finite matching corner arrays')


def calibrate_lens(dataset, *, held_out_fraction=0.2, min_views=6):
    """Fit intrinsics/distortion and report held-out pixel reprojection errors."""
    _require_deps()
    if not isinstance(dataset, ChessboardCorners):
        raise TypeError('dataset must be returned by collect_corners')
    if isinstance(min_views, bool) or not isinstance(min_views, int) or min_views < 6:
        raise ValueError('min_views must be a finite integer >= 6')
    if len(dataset.image_points) < min_views:
        raise ValueError(f'need at least {min_views} chessboard views')
    if (isinstance(held_out_fraction, bool) or not isinstance(held_out_fraction, (int, float))
            or not np.isfinite(held_out_fraction) or not 0 < held_out_fraction < 0.5):
        raise ValueError('held_out_fraction must be between 0 and 0.5')
    _validate_dataset(dataset)
    held_count = max(2, int(round(len(dataset.image_points) * held_out_fraction)))
    if len(dataset.image_points) - held_count < 4:
        held_count = len(dataset.image_points) - 4
    held_count = max(1, held_count)
    split = len(dataset.image_points) - held_count
    train_obj, held_obj = dataset.object_points[:split], dataset.object_points[split:]
    train_img, held_img = dataset.image_points[:split], dataset.image_points[split:]
    _check_diversity(train_img, dataset.image_size)
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        list(train_obj), list(train_img), dataset.image_size, None, None)
    fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    width, height = dataset.image_size
    if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or not np.isfinite(rms)
            or fx <= 0 or fy <= 0 or fx > width * 20 or fy > height * 20
            or cx < -width or cx > 2 * width or cy < -height or cy > 2 * height):
        raise ValueError('OpenCV returned non-finite calibration')

    def errors(objects, images, rv, tv):
        result = []
        for obj, observed, r, t in zip(objects, images, rv, tv):
            predicted, _ = cv2.projectPoints(obj, r, t, matrix, distortion)
            result.append(float(np.sqrt(np.mean(np.sum((predicted - observed) ** 2, axis=2)))))
        return tuple(result)

    train_errors = errors(train_obj, train_img, rvecs, tvecs)
    held_errors = []
    for obj, observed in zip(held_obj, held_img):
        ok, r, t = cv2.solvePnP(obj, observed, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError('held-out chessboard pose solve failed')
        held_errors.extend(errors((obj,), (observed,), (r,), (t,)))
    return LensCalibrationReport(tuple(matrix.tolist()), tuple(distortion.reshape(-1).tolist()),
                                dataset.image_size, float(rms), train_errors, tuple(held_errors),
                                dataset.frame_indices[:split], dataset.frame_indices[split:])


def calibration_board_svg(pattern_size, square_size_mm=25.0, margin_mm=10.0):
    """Return printable SVG for a board whose square size is explicitly labelled."""
    cols, rows = _pattern(pattern_size)
    if (not math.isfinite(square_size_mm) or not math.isfinite(margin_mm)
            or square_size_mm <= 0 or margin_mm < 0):
        raise ValueError('board dimensions must be positive')
    width, height = (cols + 1) * square_size_mm, (rows + 1) * square_size_mm
    outer_w, outer_h = width + 2 * margin_mm, height + 2 * margin_mm
    if outer_w > 10000 or outer_h > 10000:
        raise ValueError('board dimensions are unreasonably large')
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{outer_w:g}mm" height="{outer_h:g}mm" viewBox="0 0 {outer_w:g} {outer_h:g}">',
             f'<rect width="{outer_w:g}" height="{outer_h:g}" fill="white"/>']
    for row in range(rows + 1):
        for col in range(cols + 1):
            if (row + col) % 2 == 0:
                parts.append(f'<rect x="{margin_mm + col*square_size_mm:g}" y="{margin_mm + row*square_size_mm:g}" width="{square_size_mm:g}" height="{square_size_mm:g}" fill="black"/>')
    parts.append(f'<text x="{margin_mm:g}" y="{outer_h - margin_mm/3:g}" font-size="4">{cols}x{rows} inner corners; square {square_size_mm:g} mm; print at 100%</text></svg>')
    return ''.join(parts)


def _main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', nargs='*', help='chessboard image paths or glob patterns')
    parser.add_argument('--cols', type=int, default=9)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--square-size', type=float, required=True, help='inner square size in metres')
    parser.add_argument('--board-svg', type=Path, help='write printable board SVG and exit')
    args = parser.parse_args()
    if args.board_svg:
        args.board_svg.write_text(calibration_board_svg((args.cols, args.rows), args.square_size * 1000), encoding='utf-8')
        return
    paths = sorted({p for pattern in args.images for p in glob.glob(pattern)})
    if not paths:
        parser.error('provide image paths or --board-svg')
    frames = [cv2.imread(path, cv2.IMREAD_COLOR) for path in paths]
    if any(frame is None for frame in frames):
        raise ValueError('could not read one or more image files')
    report = calibrate_lens(collect_corners(frames, (args.cols, args.rows), args.square_size))
    print(json.dumps({
        'camera_matrix': report.camera_matrix, 'distortion': report.distortion,
        'image_size': report.image_size, 'rms_error_px': report.rms_error,
        'train_errors_px': report.train_errors, 'held_out_errors_px': report.held_out_errors,
        'held_out_rms_px': report.held_out_rms, 'train_indices': report.train_indices,
        'held_out_indices': report.held_out_indices,
    }, allow_nan=False, sort_keys=True))


if __name__ == '__main__':
    _main()
