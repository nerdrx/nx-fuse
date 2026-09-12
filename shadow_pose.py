"""Bounded, read-only reconstruction of body candidates from one pose frame."""

try:
    import cv2
    import numpy as np
    import numbers
except ImportError:  # Keep module importable until camera extras are installed.
    cv2 = None
    np = None


_SOLVE = (11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)
_OUT = {
    "hip": (23, 24),
    "left_elbow": (13,), "right_elbow": (14,),
    "left_knee": (25,), "right_knee": (26,),
    "left_foot": (27,), "right_foot": (28,),
}


def _array(value, shape, name):
    if np is None:
        raise RuntimeError("numpy and OpenCV are required")
    try:
        raw = np.asarray(value, dtype=object)
        if any(not isinstance(v, numbers.Real) or isinstance(v, (bool, np.bool_)) for v in raw.flat):
            raise ValueError(f"{name} must contain finite numbers")
    except (TypeError, ValueError) as exc:
        if isinstance(exc, ValueError):
            raise
    try:
        a = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain finite numbers") from exc
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(f"{name} must be finite with shape {shape}")
    return a


def _landmarks(estimate):
    lm = estimate.get("landmarks")
    xyz = estimate.get("inferred3d")
    if not isinstance(lm, (list, tuple)) or len(lm) != 33:
        raise ValueError("landmarks must contain 33 landmarks")
    xyz = _array(xyz, (33, 3), "inferred3d")
    xy = np.empty((33, 2), dtype=float)
    vis = np.zeros(33, dtype=float)
    for i, point in enumerate(lm):
        if not isinstance(point, dict):
            raise ValueError("landmarks entries must be objects")
        try:
            if any(isinstance(point[key], (bool, np.bool_)) for key in ("x", "y", "visibility")):
                raise ValueError
            xy[i] = (float(point["x"]), float(point["y"]))
            vis[i] = float(point["visibility"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("landmarks require finite x, y, visibility") from exc
    if not np.isfinite(xy).all() or not np.isfinite(vis).all():
        raise ValueError("landmarks must be finite")
    if np.any((vis < 0) | (vis > 1)):
        raise ValueError("visibility must be in [0, 1]")
    if np.any((xy[vis >= .7] < 0) | (xy[vis >= .7] > 1)):
        raise ValueError("visible landmarks must be normalized")
    return xy, vis, xyz


def reconstruct(estimate, lens, alignment, head):
    """Return conservative VR body candidates; never returns head or hand joints."""
    if cv2 is None or np is None:
        raise RuntimeError("numpy and OpenCV are required")
    if not isinstance(estimate, dict) or not isinstance(lens, dict):
        raise ValueError("estimate and lens must be objects")
    xy, vis, model = _landmarks(estimate)
    camera = _array(lens.get("camera_matrix"), (3, 3), "camera_matrix")
    distortion_value = lens.get("distortion")
    try:
        distortion_shape = (len(distortion_value),)
    except TypeError as exc:
        raise ValueError("distortion must be a numeric vector") from exc
    distortion = _array(distortion_value, distortion_shape, "distortion")
    if distortion.size not in (4, 5, 8, 12, 14):
        raise ValueError("distortion must contain 4, 5, 8, 12, or 14 values")
    size = lens.get("image_size")
    if not isinstance(size, (tuple, list)) or len(size) != 2:
        raise ValueError("image_size must be (width, height)")
    width, height = _array(size, (2,), "image_size")
    if not np.isfinite([width, height]).all() or min(width, height) <= 0:
        raise ValueError("image_size must be positive and finite")
    if camera[0, 0] <= 0 or camera[1, 1] <= 0:
        raise ValueError("camera_matrix has invalid focal length")
    if not isinstance(alignment, dict) or not isinstance(alignment.get("camera_to_vr"), dict):
        raise ValueError("alignment must contain camera_to_vr")
    pose = alignment["camera_to_vr"]
    r_vc = _array(pose.get("rotation"), (3, 3), "camera_to_vr.rotation")
    t_vc = _array(pose.get("translation"), (3,), "camera_to_vr.translation")
    if not np.allclose(r_vc.T @ r_vc, np.eye(3), atol=2e-4) or np.linalg.det(r_vc) <= 0:
        raise ValueError("camera_to_vr.rotation must be a proper rotation")
    hp = _array(head.get("position") if isinstance(head, dict) else None, (3,), "head.position")
    if not isinstance(head, dict) or not isinstance(head.get("flags"), int) or (head["flags"] & 0x33) != 0x33:
        raise ValueError("tracked head is unavailable")
    orientation = _array(head.get("orientation"), (4,), "head.orientation")
    if not np.isclose(np.linalg.norm(orientation), 1., atol=.05):
        raise ValueError("head.orientation must be a unit quaternion")
    if vis[7] < .7 or vis[8] < .7:
        raise ValueError("both ears need visibility >= 0.7")
    chosen = [i for i in _SOLVE if vis[i] >= .7]
    if len(chosen) < 8:
        raise ValueError("need at least 8 stable torso/limb landmarks")
    points = model[chosen]
    if np.linalg.matrix_rank(points - points.mean(0), tol=1e-7) < 3:
        raise ValueError("visible model points are degenerate")
    extent = float(np.linalg.norm(points.max(0) - points.min(0)))
    if not .2 <= extent <= 4.0:
        raise ValueError("inferred model length scale is implausible")
    pixels = xy[chosen] * (width, height)
    try:
        ok, rvec, tvec = cv2.solvePnP(points.astype(float), pixels.astype(float), camera,
                                      distortion.reshape(-1, 1), flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError("pose solve failed")
        projected, _ = cv2.projectPoints(points, rvec, tvec, camera, distortion.reshape(-1, 1))
    except cv2.error as exc:
        raise ValueError("pose solve failed") from exc
    projected = projected.reshape(-1, 2)
    error = np.linalg.norm(projected - pixels, axis=1)
    rms = float(np.sqrt(np.mean(error ** 2)))
    try:
        rot, _ = cv2.Rodrigues(rvec)
    except cv2.error as exc:
        raise ValueError("pose rotation conversion failed") from exc
    cam = (rot @ model.T + tvec.reshape(3, 1)).T
    if not np.isfinite(cam).all() or np.any(cam[:, 2] <= 0) or not np.isfinite(rms) \
            or rms > 8.0 or np.max(error) > 20.0:
        raise ValueError("reprojection or depth check failed")
    head_cam = r_vc.T @ (hp - t_vc)
    try:
        head_px, _ = cv2.projectPoints(head_cam.reshape(1, 3), np.zeros(3), np.zeros(3), camera,
                                       distortion.reshape(-1, 1))
    except cv2.error as exc:
        raise ValueError("head projection failed") from exc
    ear_px = xy[[7, 8]].mean(0) * (width, height)
    assoc_limit = 80.0 * min(width, height) / 720.0
    if head_cam[2] <= 0 or float(np.linalg.norm(head_px.ravel() - ear_px)) > assoc_limit:
        raise ValueError("tracked head does not associate with visible ears")
    vr = (r_vc @ cam.T).T + t_vc
    ear_vr = vr[[7, 8]].mean(0)
    anchor_shift = hp - ear_vr
    if np.linalg.norm(anchor_shift) > .35:
        raise ValueError("head anchor differs from reconstructed ears")
    vr += anchor_shift
    joints = {}
    for name, indices in _OUT.items():
        if any(vis[i] < .7 for i in indices):
            continue
        joints[name] = {"position": vr[list(indices)].mean(0).tolist(),
                        "confidence": float(min(vis[i] for i in indices))}
    if len(joints) < 3:
        raise ValueError("insufficient visible body candidates")
    return {"joints": joints, "rms_px": rms, "anchor_shift_m": float(np.linalg.norm(anchor_shift)),
            "note": "Approximate anatomical ear-to-HMD anchoring; model-inferred body is uncalibrated."}
