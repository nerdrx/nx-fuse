# Development validation

Performed 2026-09-12 on the development Linux host. These are functional
checks, not evidence of improved tracking accuracy or VR performance.

- Python suite: 49 checks pass in the camera environment. Fusion/replay cover
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

- New lens calibration tests use known camera geometry, varied projected boards,
  JPEG corner collection, held-out reprojection, duplicate/invalid-view rejection
  and native workflow reset. No physical chessboard calibration was performed.
- The read-only WiVRn tap compiles with the server. A real C++ serializer sends
  fixture datagrams to a Unix socket and the Python decoder checks routes,
  roles and generations. Conversion/clock helpers are stubbed in this fixture;
  live clock synchronization remains untested. Receiver tests cover packet
  rejection, sequence/generation changes, expiry and socket ownership.
- The tracking-lab launcher test verifies that its owned worker stops and
  removes its socket when a fake dashboard exits. Native headless inspection
  displayed an explicitly synthetic tap packet and expanded lens controls.

- Spatial alignment tests cover a known rotated/distorted camera, inverse
  transform, behind-camera rejection, degenerate samples and held-out bad clicks.
  Workflow/HTTP checks cover frozen tokens, bounded image selection, camera/lens/
  generation invalidation and pose collection using the actual receiver history.
- Anchor history tests cover stationary timing, measured quaternion offsets,
  motion/rotation rejection, invalid tracking and recenter. Camera tests verify
  atomic JPEG/sequence/arrival metadata and restart identity.
- Headless native alignment UI used a labeled synthetic camera fixture: freeze
  displayed its JPEG, a letterbox click was rejected, and clicking its reference
  added exactly one sample. No physical camera was opened for this check.

Earlier web-console checks covered desktop/mobile layout, assisted and
camera-only simulation, occlusion, and front/side debug views.

No live Pico tracking session, camera-to-VR calibration, pose injection,
wearer association, physical bed coverage or measured tracking improvement
has been demonstrated. Native camera estimates remain separate from the
synthetic fusion output. Camera frames are not recorded or uploaded.
