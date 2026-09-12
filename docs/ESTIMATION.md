# Optional single-frame pose estimation

`estimator.py` exposes `PoseEstimator(model_path)` for one BGR `numpy.uint8`
frame. The model path is mandatory and must point to an existing MediaPipe
Pose Landmarker `.task` bundle. The module never downloads a model.

```python
from estimator import PoseEstimator

with PoseEstimator('/models/pose_landmarker.task') as estimator:
    estimate = estimator.estimate(frame_bgr)
```

`estimate` returns `None` when no pose is found, the result is malformed, or
more than one person is detected. Otherwise it returns `PoseEstimate`:
`landmarks` contains 33 normalized image landmarks (`x`, `y`, and optional
`visibility`), while `inferred3d` contains hip-relative MediaPipe
model-inferred metric world `(x, y, z)` landmarks. These coordinates are
uncalibrated to VR and are **not measured camera depth**; do not pass them to
fusion as calibrated coordinates.

The wrapper uses MediaPipe Tasks image mode and configures `num_poses=2` so an
ambiguous multi-person frame can be rejected. MediaPipe's official API accepts
an explicit `BaseOptions(model_asset_path=...)`, and `PoseLandmarker.detect`
consumes a single `mp.Image` in image mode:

- [PoseLandmarker Python API](https://ai.google.dev/edge/api/mediapipe/python/mp/tasks/vision/PoseLandmarker)
- [PoseLandmarker options](https://ai.google.dev/edge/api/mediapipe/python/mp/tasks/vision/PoseLandmarkerOptions)
- [PoseLandmarker result](https://ai.google.dev/edge/api/mediapipe/python/mp/tasks/vision/PoseLandmarkerResult)
