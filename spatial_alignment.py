"""Fit a calibrated camera pose against tracked VR anchors.

The fitted OpenCV pose maps VR coordinates into camera coordinates as
``x_camera = R @ x_vr + t``.  The public ``camera_to_vr`` value is its inverse:
``x_vr = R.T @ (x_camera - t)``.  All returned values are ordinary Python
scalars/lists so the result can be passed directly to :mod:`json`.
"""

import numbers

try:
    import cv2
    import numpy as np
except ImportError:  # Keep nx-fuse importable without camera extras.
    cv2 = None
    np = None


def _require_deps():
    if cv2 is None or np is None:
        raise RuntimeError("numpy and OpenCV are required (install camera extras)")


def _numbers(value, name):
    values = np.asarray(value, dtype=object)
    if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, numbers.Real) for v in values.flat):
        raise ValueError(f"{name} must contain numbers, not booleans or text")
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must contain only finite values")
    return result


def _array(value, shape, name):
    result = _numbers(value, name)
    if result.ndim != 2 or result.shape[1] != shape or result.shape[0] < 1:
        raise ValueError(f"{name} must be an N x {shape} array")
    return result


def _spread(points, name):
    centered = points - points.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    if len(singular) < 3 or singular[0] <= 1e-9 or singular[-1] <= singular[0] * 1e-3:
        raise ValueError(f"training {name} are degenerate; need diverse non-coplanar captures")


def _pixel_errors(world, image, camera, distortion, rvec, tvec):
    try:
        projected, _ = cv2.projectPoints(world, rvec, tvec, camera, distortion)
    except cv2.error as exc:
        raise ValueError("OpenCV reprojection failed") from exc
    return np.linalg.norm(projected.reshape(-1, 2) - image, axis=1)


def solve_alignment(world_points, image_points, camera_matrix, distortion, image_size):
    """Return a strict, JSON-safe VR-to-camera alignment report.

    At least twelve matched anchors are required.  The final approximately 25%
    (at least three) are held out in input order; this intentionally catches
    bad clicks or a pose that only fits the training captures.  ``validated``
    is true only when both train and held-out errors meet the 3 px RMS/6 px
    maximum limits.  A failed error gate returns a report with ``validated``
    false so callers can display it; malformed or geometrically degenerate
    input raises ``ValueError``.
    """
    _require_deps()
    world = _array(world_points, 3, "world_points")
    image = _array(image_points, 2, "image_points")
    if world.shape[0] != image.shape[0] or world.shape[0] < 12:
        raise ValueError("need at least 12 matched world/image samples")
    camera = _numbers(camera_matrix, "camera_matrix")
    if camera.shape != (3, 3) or not np.isfinite(camera).all():
        raise ValueError("camera_matrix must be a finite 3 x 3 matrix")
    distortion = _numbers(distortion, "distortion").reshape(-1, 1)
    if distortion.size not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
        raise ValueError("distortion must contain 4, 5, 8, 12, or 14 finite coefficients")
    if (not isinstance(image_size, (tuple, list)) or len(image_size) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in image_size)):
        raise ValueError("image_size must be (width, height)")
    width, height = float(image_size[0]), float(image_size[1])
    if not np.isfinite([width, height]).all() or width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive finite dimensions")
    fx, fy = camera[0, 0], camera[1, 1]
    if (fx <= 0 or fy <= 0 or abs(camera[0, 1]) > 1e-9 or abs(camera[1, 0]) > 1e-9
            or not np.allclose(camera[2], [0, 0, 1], atol=1e-9, rtol=0)
            or fx > width * 20 or fy > height * 20
            or camera[0, 2] < -width or camera[0, 2] > 2 * width
            or camera[1, 2] < -height or camera[1, 2] > 2 * height):
        raise ValueError("camera_matrix has implausible calibration values")
    if (np.any(image[:, 0] < 0) or np.any(image[:, 0] >= width)
            or np.any(image[:, 1] < 0) or np.any(image[:, 1] >= height)):
        raise ValueError("image_points must lie inside image bounds")

    held_count = max(3, int(np.ceil(world.shape[0] * 0.25)))
    split = world.shape[0] - held_count
    if split < 6:
        raise ValueError("need at least six training samples")
    train_world, held_world = world[:split], world[split:]
    train_image, held_image = image[:split], image[split:]
    _spread(train_world, "3D points")
    if np.ptp(train_image, axis=0).min() < min(width, height) * 0.05:
        raise ValueError("training image points are not sufficiently spread")

    try:
        ok, rvec, tvec = cv2.solvePnP(train_world.astype(np.float64), train_image.astype(np.float64),
                                      camera, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    except cv2.error as exc:
        raise ValueError("OpenCV pose solve failed") from exc
    if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        raise ValueError("OpenCV pose solve failed")
    try:
        rotation, _ = cv2.Rodrigues(rvec)
    except cv2.error as exc:
        raise ValueError("OpenCV rotation conversion failed") from exc
    camera_points = (rotation @ world.T + tvec.reshape(3, 1)).T
    if not np.isfinite(rotation).all() or not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
        raise ValueError("solved pose violates cheirality")
    train_error = _pixel_errors(train_world, train_image, camera, distortion, rvec, tvec)
    held_error = _pixel_errors(held_world, held_image, camera, distortion, rvec, tvec)
    train_rms = float(np.sqrt(np.mean(train_error ** 2)))
    held_rms = float(np.sqrt(np.mean(held_error ** 2)))
    return {
        "camera_to_vr": {
            "rotation": rotation.T.tolist(),
            "translation": (-rotation.T @ tvec.reshape(3)).tolist(),
        },
        "training_rms_px": train_rms,
        "held_out_rms_px": held_rms,
        "max_training_px": float(np.max(train_error)),
        "max_held_out_px": float(np.max(held_error)),
        "validated": bool(train_rms <= 3.0 and held_rms <= 3.0
                           and np.max(train_error) <= 6.0 and np.max(held_error) <= 6.0),
    }
