"""Optional, single-frame MediaPipe pose estimation.

This module does not download models or connect cameras.  Pass an explicit
MediaPipe Pose Landmarker task model path before calling :class:`PoseEstimator`.
"""
from dataclasses import dataclass
import math
from pathlib import Path

try:
    import numpy as np
except ImportError:  # Keep fusion/replay imports usable without NumPy.
    np = None


LANDMARK_NAMES = (
    'nose', 'left_eye_inner', 'left_eye', 'left_eye_outer', 'right_eye_inner',
    'right_eye', 'right_eye_outer', 'left_ear', 'right_ear', 'mouth_left',
    'mouth_right', 'left_shoulder', 'right_shoulder', 'left_elbow',
    'right_elbow', 'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky',
    'left_index', 'right_index', 'left_thumb', 'right_thumb', 'left_hip',
    'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle',
    'left_heel', 'right_heel', 'left_foot_index', 'right_foot_index',
)
HIP_INDICES = (23, 24)


@dataclass(frozen=True)
class PoseEstimate:
    """One-person pose in camera-image coordinates.

    ``landmarks`` contains normalized image ``x``/``y`` and optional visibility.
    ``inferred3d`` is hip-relative model-inferred metric world ``(x, y, z)``;
    it is uncalibrated to VR and is not measured camera depth.
    """

    landmarks: tuple[dict, ...]
    inferred3d: tuple[tuple[float, float, float], ...]
    coordinate_note: str = 'hip-relative MediaPipe model-inferred metric world coordinates; NOT calibrated to VR'


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class PoseEstimator:
    """Lazy MediaPipe Tasks Pose Landmarker wrapper for one BGR frame."""

    def __init__(self, model_path, *, landmarker_factory=None):
        path = Path(model_path) if isinstance(model_path, (str, Path)) else None
        if path is None or not path.is_file():
            raise ValueError('model_path must name an existing model file')
        self.model_path = path
        self._factory = landmarker_factory
        self._landmarker = None

    def _ensure_landmarker(self):
        if self._landmarker is not None:
            return self._landmarker
        if self._factory is not None:
            self._landmarker = self._factory(self.model_path)
            return self._landmarker
        try:
            import mediapipe as mp
            base = mp.tasks.BaseOptions(model_asset_path=str(self.model_path))
            options = mp.tasks.vision.PoseLandmarkerOptions(
                base_options=base, num_poses=2,
                min_pose_detection_confidence=.5,
                min_pose_presence_confidence=.5,
                min_tracking_confidence=.5,
            )
            self._landmarker = mp.tasks.vision.PoseLandmarker.create_from_options(options)
        except ImportError as exc:
            raise RuntimeError('mediapipe is required for PoseEstimator') from exc
        return self._landmarker

    def estimate(self, frame_bgr):
        """Return one pose, or ``None`` when no unique valid person is present."""
        if np is None:
            raise RuntimeError('numpy is required for PoseEstimator.estimate')
        if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError('frame_bgr must be HxWx3 numpy array')
        if frame_bgr.dtype != np.uint8 or frame_bgr.shape[0] == 0 or frame_bgr.shape[1] == 0:
            raise ValueError('frame_bgr must be non-empty uint8 image')
        landmarker = self._ensure_landmarker()
        if self._factory is not None:
            result = landmarker.detect(frame_bgr)
        else:
            import mediapipe as mp
            rgb = np.ascontiguousarray(frame_bgr[:, :, ::-1])
            result = landmarker.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
        poses = getattr(result, 'pose_landmarks', None)
        if not poses or len(poses) != 1 or len(poses[0]) != len(LANDMARK_NAMES):
            return None
        raw = poses[0]
        if any(not all(_finite(getattr(point, axis, None)) for axis in ('x', 'y', 'z'))
               or any(getattr(point, field, None) is not None
                      and (not _finite(getattr(point, field)) or not 0 <= getattr(point, field) <= 1)
                      for field in ('visibility', 'presence'))
               for point in raw):
            return None
        world = getattr(result, 'pose_world_landmarks', None)
        if not world or len(world) != 1 or len(world[0]) != len(LANDMARK_NAMES):
            return None
        world = world[0]
        if any(not all(_finite(getattr(point, axis, None)) for axis in ('x', 'y', 'z')) for point in world):
            return None
        hips = tuple(sum(getattr(world[index], axis) for index in HIP_INDICES) / 2 for axis in ('x', 'y', 'z'))
        landmarks = []
        inferred = []
        for index, (name, point) in enumerate(zip(LANDMARK_NAMES, raw)):
            item = {'name': name, 'x': float(point.x), 'y': float(point.y)}
            if _finite(getattr(point, 'visibility', None)):
                item['visibility'] = float(point.visibility)
            landmarks.append(item)
            world_point = world[index]
            inferred.append(tuple(float(getattr(world_point, axis) - hips[i])
                                  for i, axis in enumerate(('x', 'y', 'z'))))
        return PoseEstimate(tuple(landmarks), tuple(inferred))

    def close(self):
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
