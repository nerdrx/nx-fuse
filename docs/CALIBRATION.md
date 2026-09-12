# Offline rigid calibration

`calibration.py` fits a rigid transform from matched model points to target
3D points. It estimates rotation and translation only. It never estimates
scale and never applies the result to VR.

```python
from calibration import fit_rigid
report = fit_rigid(model_points, target_points,
                   validation_model_points, validation_target_points)
```

At least four finite, noncoplanar, well-conditioned points are required.
`report.rms_error` and `report.max_error` describe fit samples;
`report.held_out_errors` reports each validation sample separately. Large fit
error usually means wrong units, bad correspondences, or a model-scale error.
A rigid transform cannot solve model-scale errors.

This helper is offline math, not completed camera calibration. Hip-relative
MediaPipe landmarks move with the subject and cannot directly define camera
extrinsics. Collect correspondences in a stable camera coordinate system with
known targets (or another fixed reference) before fitting.

CLI input uses explicit metadata and matched pairs:

```json
{
  "camera_id": "cam-left",
  "resolution": [1280, 720],
  "session_generation": "2026-09-12-a",
  "fit": [
    {"model": [0, 0, 0], "target": [1, 2, 3]},
    {"model": [1, 0, 0], "target": [2, 2, 3]},
    {"model": [0, 1, 0], "target": [1, 3, 3]},
    {"model": [0, 0, 1], "target": [1, 2, 4]}
  ],
  "validation": [{"model": [0.5, 0.5, 0.5], "target": [1.5, 2.5, 3.5]}]
}
```

Save this synthetic example as `pairs.json`, then run
`python3 calibration.py pairs.json`. Real validation requires independent
measurements; this example only demonstrates the file format.

The CLI emits a JSON calibration profile containing the transform, fit /
held-out errors, and `validated`. `validated` is false when held-out RMS or
maximum error exceeds the default limits. Collect points offline and review
errors before any runtime integration.
