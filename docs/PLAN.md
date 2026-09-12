# NX Fuse: camera-assisted Pico full-body tracking

## Outcome and scope

Improve visible hip, knee and foot positions using room cameras, while
retaining Pico continuity and WiVRn NX's headset/controller tracking.
Support one camera first, then independent room and overhead-bed cameras.
Keep the fusion core tracker-neutral; Pico is the first hardware validation
target. Other tracker adapters remain untested candidates until verified on
their own hardware. See `TRACKERS.md` for normalization and acceptance rules.
Support an explicit camera-only mode without body trackers. One RGB camera
is a required design target through monocular 3D estimation; additional
cameras improve coverage/geometry. See `MONOCULAR.md` for candidate research,
calibration, headset anchoring, unavailable-joint behavior and acceptance gates.
Deliver a clear, attractive dashboard that shows when assistance is real,
why a joint fell back, and how to return to Pico instantly.

This delivery is a development framework and integration plan. Synthetic
poses and camera registration are not computer vision or live WiVRn
tracking. No measured tracking improvement, working headset attachment or
hardware validation is claimed. See `WIVRN-INTEGRATION.md` for the inspected
source and proposed hooks. The existing WiVRn installation remains intact.

## Architecture

1. Capture workers produce timestamped frames without growing queues.
2. A pose estimator produces per-person joint observations and uncertainty.
3. Calibration maps metric observations into the current WiVRn space.
4. Time alignment compares observations against raw Pico history.
5. Fusion selects plausible per-joint corrections, with bounded strength,
   hysteresis and deterministic expiry.
6. A small WiVRn adapter applies fresh snapshots before tracker fan-out.
7. The dashboard reads diagnostics separately from the tracking path.

Start with position correction and original Pico orientations. Do not
equate high pose-network confidence with metric accuracy. Keep raw inputs,
derived observations and final outputs separate for debugging and replay.

## Milestones and acceptance gates

| Milestone | Work | Acceptance |
|---|---|---|
| 0. Offline foundation | Local dashboard, explicit demo state, pose contract, deterministic fusion/replay checks, plan and adapter map. | Runs without headset/cameras; tests cover stale/invalid input and bounded handover; never advertises demo as live. |
| 1. Live baseline tap | Confirm actual BD/HTC route, obtain raw body/head/controller observations and clock mapping; record opted-in pose data. | Log negotiated route and role map; no pose modifications; no VR timing regression; reconnect creates a new session generation. |
| 2. One-camera observation | Inventory connected devices; select supported resolution; add one estimator backend, target association and calibration wizard. | Held-out calibration error reported; side/back/lying confidence failures visible; unplugging camera cannot affect baseline VR. |
| 3. Shadow correction | Align camera exposures to Pico history and calculate residuals without applying them. | Replay identical logs deterministically; compare raw/fused errors, latency and handover; reject identity swaps and stale calibration. |
| 4. Opt-in live assistance | Bounded WiVRn snapshot adapter; hip first, then feet/knees; status and immediate disable. | Actual game receives corrected poses; body and virtual outputs agree; baseline returns on every injected failure; headset/controllers unchanged. |
| 5. Multiple cameras and bed | Per-camera extrinsics and timing; overlap fusion plus non-overlap handover; overhead-view evaluation. | Room-to-bed transitions avoid jumps/identity changes; one failed camera only removes its contribution; occluded joints fall back. |
| 6. Product pass | Guided setup, saved camera profiles, calibration health, A/B replay, accessibility, launcher and concise troubleshooting. | New session cannot use stale calibration silently; keyboard workflow works; representative live session completes without intervention. |

Proceed to the next milestone only with evidence from its gate. Do not
build extra model backends, remote phone streaming or an orientation solver
until the basic correction demonstrates benefit.

## Experiments and measurable targets

Run paired baseline/assisted trials using the same recording where possible,
then verify live. Include standing still, slow crouch, walking, quick foot
movement, sitting, lying on back/side, crossed legs, partial blanket cover,
room-to-bed transition and another person entering view. Obtain consent
before capturing another person; keep bedroom images local and recording
off by default. Pose-only diagnostics should be the default artifact.

Measure stationary jitter (mm), foot sliding during annotated contact,
bone-length variation, held-out positional error, capture-to-output delay,
correction age, per-joint coverage, transition displacement, CPU/GPU usage
and VR frame times. Ground truth can start with known static target
positions and annotated contacts; lower jitter alone does not prove more
accurate motion. Dynamic accuracy needs an independent reference.

Initial go/no-go targets, to adjust after baseline measurement:

- At least 20% lower hip/foot stationary jitter on the selected visible
  trial, without increased positional bias beyond calibration uncertainty.
- No more than 2 cm extra one-frame displacement attributable to routine
  confidence handover in slow-motion trials; measure fast movement separately.
- Normal camera loss returns exactly to baseline within 0.5 s. Invalid
  space/session generation must bypass correction immediately.
- No material VR frame-time regression; adapter p99 under 0.1 ms on target
  host. Camera processing must remain outside callbacks.
- Every reported result includes hardware, camera mode, calibration version,
  WiVRn revision, estimator/settings, trial length and uncertainty.

These are engineering targets, not achieved benchmark numbers. Camera
exposure/transport latency and model performance must be measured before
choosing operational stale thresholds.

## Multi-camera and overhead details

Each camera has its own capture timing, intrinsics, distortion, extrinsics
and uncertainty. Shared overlap allows triangulation only after calibration,
synchronization and person/joint correspondence checks. Independent cameras
covering separate areas can still hand over through a shared tracking space;
they cannot triangulate a person neither sees simultaneously. Never average
inconsistent coordinate systems or force agreement between occluded views.

For the bed view, test rotated image inference and overhead-specific
performance before choosing a model. Bed surface height is distinct from
floor height: floor-contact heuristics must not pull a reclining person
downward. Blankets and hidden limbs require fallback, not invented
confidence. Retain Pico orientation initially: cameras that locate an ankle
do not necessarily observe foot twist. Explicitly associate the headset
wearer; ambiguous identities mean no camera correction.

## Dashboard requirements

Use a calm dark interface, clear violet/cyan source colors, readable labels
and a large baseline/assisted comparison. Distinguish Demo, Observation,
Shadow and Live states in text. Show raw and corrected skeletons, per-joint
source/confidence/age, camera health and calibration status. Offer guided
camera setup and calibration, a diagnostics drawer, and one clear disable
control. Never show a green connected headset from synthetic data. Do not
hide errors in color alone; controls need visible focus, labels and
keyboard access. No camera should start or record merely by opening the UI.

### Camera and estimation debug views

The framework debug panels show synthetic camera observations separately from
final poses, with joint position/depth, confidence and sample age. They are not
real video or model output. The real capture milestone must replace this with:

- A selectable preview for each camera with 2D keypoints, body bounding region,
  association identity and rejected/occluded landmarks over the actual image.
- A separate estimated 3D body view, labeled as inferred depth, alongside raw
  tracker and final fused skeletons. Keep camera coordinates distinct from VR
  coordinates and show the calibration transform/version.
- A per-joint inspector: source camera, observation age, detector confidence,
  geometric/depth uncertainty, raw position, correction, final position and
  explicit acceptance/rejection reason. Confidence is not accuracy.
- Pause/scrub for opted-in recordings; freezing the debug display must never
  freeze or replay live VR output. Pose-only logging by default, image recording
  only when deliberately enabled. Debug work stays outside tracking callbacks.
- Pop-out or separate-window views later if useful during calibration; native
  browser tabs already allow the current console to be placed on another screen.

## Open research risks

- The runtime may expose generic HTC tracker poses rather than BD joints;
  their body role association must be proven, not inferred from order.
- Single-view depth, body scale and overhead poses can be systematically
  wrong despite high detector confidence.
- Delayed absolute pose blending can damage otherwise responsive tracking;
  residual correction is a hypothesis requiring A/B comparison.
- Cameras share host GPU resources with VR; lower-rate inference may be
  preferable to the most accurate model.
- Correct joint positions do not guarantee sensible avatar IK or tracker
  orientation. Validate the receiving game's behavior directly.
- Recenter and calibration drift can create large plausible-looking errors;
  generation checks and held-out calibration validation are required.

## Next concrete work when hardware is available

Read live negotiated body type and tracker roles; inventory connected
cameras without enabling capture; select one room view and measure its
latency. Implement a read-only WiVRn pose tap in an isolated worktree, then
calibrate and run shadow trials. Only apply live corrections after those
trials show improvement. A useful first demo is one visible hip whose
position is steadier while its raw Pico fallback survives camera unplugging.
