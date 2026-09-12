# Development validation

Performed 2026-09-12 on the development Linux host. These are functional
checks, not evidence of improved tracking accuracy or VR performance.

- Python suite: 29 checks pass in the camera environment. Fusion/replay cover
  stale/future/nonfinite/malformed observations, source disagreement, exact
  fallback expiry, protected head/hands and camera-only missing joints.
- Fake camera tests cover explicit capture, latest JPEG, duplicate start,
  restart, failed capture, bounded estimates and disabling during inference.
  Model construction, inference and shutdown stay on the capture thread;
  results from an old enable/disable generation are discarded.
- Physical smoke: `/dev/video0` (LifeCam Studio) and `/dev/video2` (integrated
  RGB camera) both produced JPEGs while open together. First frames took about
  844 ms and 1262 ms including opening/configuration. Both capture threads
  stopped afterward. No images were saved. The V4L2 backend emitted two
  `VIDIOC_QBUF: Bad file descriptor` cleanup warnings; repeated/reliability and
  unplug tests on these physical devices remain outstanding.
- MediaPipe 1.0.1 Pose Landmarker Lite ran on the official `pose.jpg` fixture:
  33 landmarks, about 12–15 ms warm single-frame inference in this short smoke.
  First inference included roughly 322–393 ms initialization. The integrated
  preview also produced matched JPEG overlays and hip-relative inferred 3D.
  A static fixture is not a latency, motion-accuracy or body-coverage benchmark.
- Offline rigid calibration checks cover known transforms, noise, reflection,
  degenerate geometry and invalid metadata. No real calibration was performed.
- Native WiVRn NX dashboard compiled successfully. Headless gamescope checks
  covered navigation, camera fixture rendering, detached debug window, body
  simulation and a narrower window. Qt bridge tests covered controls,
  coalescing, external-worker survival, malformed/wrong-mode/oversized replies,
  stalled requests and reconnect recovery using an isolated fake service.

Earlier web-console checks covered desktop/mobile layout, assisted and
camera-only simulation, occlusion, and front/side debug views.

No live Pico tracking session, camera-to-VR calibration, pose injection,
wearer association, physical bed coverage or measured tracking improvement
has been demonstrated. Native camera estimates remain separate from the
synthetic fusion output. Camera frames are not recorded or uploaded.
