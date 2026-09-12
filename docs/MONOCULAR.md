# One camera, estimated 3D

Single-camera depth estimation is an explicit NX Fuse target, including a mode
with no body trackers. It is not implemented in the current simulation. A
second camera is an optional improvement, not a prerequisite in the design.

## Research starting points

- [MediaPipe Pose Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker)
  estimates 33 three-dimensional body landmarks and offers live-stream input.
  Evaluate it first as a lightweight observation baseline; its model coordinates
  still need calibration and validation against the VR tracking space.
- [WHAM](https://wham.is.tue.mpg.de/) reconstructs world-grounded 3D human motion
  from video. Investigate its temporal motion and global reconstruction ideas
  as a research comparison.
- [GVHMR](https://zju3dv.github.io/gvhmr/) addresses global motion recovery from
  monocular video using gravity-view coordinates. Its advertised network timing
  excludes preprocessing and uses a video sequence; that is not proof of live
  camera-to-headset latency. Evaluate causal operation and complete pipeline cost.

These are primary research/documentation references checked on 2026-09-12, not
dependencies selected or installed. Check code, model-weight and body-model
licenses separately before redistribution. Use local measurements to choose a
model rather than a paper's throughput number alone.

## Proposed first implementation

1. Use one fixed RGB camera and the headset/controller tracking already present.
   Run a causal pose estimator on the newest frame; do not queue old frames.
2. Calibrate camera intrinsics/extrinsics with visible tracked targets and known
   offsets. Headset pose is a valuable anchor after calibration, but its sensor
   origin is not an observed nose landmark and alone does not solve every
   camera transform. An obscured face needs a different observable target.
3. Estimate body-relative 3D pose and fit measured body proportions/height.
   Recover scale/root translation using calibrated geometry, available tracked
   anchors and temporal constraints. Treat inferred depth as uncertain rather
   than as a depth-sensor measurement.
4. Stabilize with fixed body lengths, motion continuity and explicit contact
   hypotheses. Keep floor and bed surfaces separate. Do not force every pose to
   stand upright or assume unseen feet touch the floor.
5. In assisted mode, compare the time-aligned estimate with original tracker
   history. In camera-only mode, publish only sufficiently valid body estimates
   and preserve the headset/controllers independently.

Body-relative depth can look plausible while global placement drifts. Evaluate
root position and limb depth separately; distinguish left/right swaps, scale
breathing, lag, orientation uncertainty and true motion. Learned image depth
could supply another prior, but does not remove metric-scale ambiguity by itself.

## Camera-only loss policy

No body trackers means no body-tracker fallback. The framework's `camera_only`
function accepts calibrated observations and optional headset/controller anchors;
it returns only fresh, mutually consistent joint positions. Missing joints are
absent. It does not retain an old body as valid or synthesize orientation.

The simulator demonstrates complete camera loss by leaving just its simulated
head and hand anchors. A live adapter must map absent body joints to invalid
tracker output or the receiving runtime's explicitly supported reduced-body
mode. Verify behavior in the actual game. Do not silently switch modes merely
because a tracker disconnects; camera-only is an explicit choice.

## Acceptance additions

- Demonstrate one-camera 3D output without body trackers, with known static
  distances and held-out poses. Keep a separate accuracy result from an
  assisted-Pico trial.
- Test forward/backward steps, turning, crouching, sitting, lying on back/side,
  overhead camera rotation, covered limbs, person swaps and reacquisition.
- Measure full capture-to-output delay and VR frame-time impact. No future
  frames may be required in the operational live path.
- Reject low-confidence depth and preserve valid headset/controller output
  after camera loss. Never present a frozen body as newly observed tracking.
- Add a second calibrated camera later and quantify improvement using matched
  trials; non-overlapping room/bed views provide coverage rather than stereo.
