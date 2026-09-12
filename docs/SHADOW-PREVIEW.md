# Headset-anchored body preview

The native **Shadow body preview** estimates visible hip, elbow, knee and
ankle positions in the camera's aligned VR space. It can run without body
trackers: the headset is required, body-tracker baselines are optional.
The preview never injects poses into WiVRn or changes its headset/controllers.

## Run

1. Use the Tracking Lab launcher and connect the headset.
2. Start one camera. Load a matching saved lens profile or complete lens setup.
3. Complete spatial alignment for the current camera/VR session.
4. Turn on body estimation, open **Shadow body preview**, and enable it.
5. Inspect cyan estimated candidates and amber raw tracker positions. Front/side
   views share the same coordinate frame. Missing or expired observations clear.

Lens profiles can be loaded explicitly through **Load lens profile…**. Their
geometry and held-out metrics are checked again; imported `validated` flags
are ignored. Match the original camera, resolution, focus and zoom. Loading a
lens file does not open a camera or restore camera-to-VR extrinsics.

## Reconstruction and limits

A single-person MediaPipe estimate supplies 2D image landmarks and model-inferred
hip-relative 3D shape. A calibrated perspective solve locates that shape in
the camera, then the spatial alignment transforms it into VR coordinates.
At least eight visible torso/limb points and nondegenerate geometry are needed.
Reprojection RMS must be at most 8 px and individual error at most 20 px;
points behind the camera and implausible scale are rejected.

The inferred ear midpoint must associate with the tracked headset in image
space and lie within 35 cm of it in reconstructed 3D. A translation then places
that midpoint at the headset origin. This is an approximate anatomical anchor,
not a measured ear-to-headset offset. Model shape/depth, occlusion, unusual
postures and headset-covered faces can still be wrong. Conservative rejection
may leave no preview in those cases. Visibility is a model score, not accuracy.

Raw tracking uses the nearest host-mapped sample within 60 ms of camera frame
arrival. It is never replaced with the newest unrelated pose or extrapolated
by Fuse. Camera exposure latency is still unknown. A 500 ms freshness limit
and context checks prevent old estimates surviving loss of tracking, camera
restart, changed lens/alignment, or recenter. Reconstruction runs outside the
camera capture and WiVRn callbacks, with bounded latest-frame work.

The displayed `left_foot`/`right_foot` candidates are model ankle locations.
They need not coincide with a physical foot tracker's origin. Residuals are
position differences, not ground-truth error or evidence of improved tracking.
No orientation correction, multi-camera fusion, or VR output is implemented
in this preview. Only one calibrated camera is selected at a time.

## Pose-only diagnostics

**Start pose log** explicitly records accepted shadow observations in memory.
It stores no JPEGs or video, stops at 600 frames, and can be stopped, downloaded
or cleared. Disabling the preview stops recording. Closing the worker discards
unexported logs. Exported JSON includes camera/session identifiers, timestamps,
body estimates, available raw baselines and diagnostic metrics.

```sh
python3 shadow_report.py nx-fuse-shadow.json > report.json
```

The standard-library report checks input size, finite values and timestamp
ordering, groups frames by camera/session and computes median, p95 and RMS
position differences. Missing body-tracker baselines stay unavailable rather
than being reported as zero error. Do not interpret these differences as
accuracy improvement without independent ground truth and paired trials.

## Verified so far

Known synthetic geometry tests cover reconstruction, nonzero head anchoring,
invalid inputs and failure cases. Worker tests cover disable-during-computation,
expiry, generation mismatch and the recording cap. The official MediaPipe pose
image produced seven candidates through the actual estimator/reconstruction
path using nominal lens values and a synthetic headset reference (about 7.16 px
reprojection RMS in that one fixture). That is a compatibility smoke test,
not a physical calibration, latency benchmark or tracking-quality measurement.
