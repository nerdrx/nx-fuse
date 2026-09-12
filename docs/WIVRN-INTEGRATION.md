# WiVRn NX integration map

Inspected 2026-09-12: [nerdrx/wivrn-nx](https://github.com/nerdrx/wivrn-nx), HEAD
`79a653899346e889f970210f3739f54550895c5b`. The inspected tracking files were
unchanged relative to that revision. No WiVRn files were modified.
Pointers below are checkout-relative, one-based lines at inspection time.
This is an implementation design, not a claim that NX Fuse is connected.

## Verify the actual route first

The headset model does **not** determine the packet route. In
`client/xr/body_tracker.cpp:27`, FB/META capability checks precede HTC checks
at line 37, and HTC precedes the Pico BD check at line 41. A Pico exposing
the HTC tracker extensions can therefore use generic tracker poses.
Record negotiated `body_type`, active tracker count, roles and extension
list from the user's live session before enabling any adapter.

| Stage | Verified source | Integration consequence |
|---|---|---|
| Pico joint acquisition | `client/xr/pico_body_tracker.cpp:27`, `:43` | Requests full BD joint set; returns position, packed orientation, per-joint flags and aggregate `all_tracked`. No per-joint probabilistic confidence. |
| Generic acquisition | `client/xr/htc_body_tracker.cpp:103` | Ordered tracker poses, not a named body skeleton. Do not guess index-to-body mapping. |
| Packet timestamps | `client/scenes/stream_tracking.cpp:518` | `timestamp` is requested pose time; `production_timestamp` is production time. These differ and can involve prediction. |
| Wire definitions | `common/wivrn_packets.h:790`, `:805` | BD carries the joint array; HTC carries up to 16 poses with velocities and flags. |
| Device setup | `server/driver/wivrn_session.cpp:261` | BD creates a body device plus virtual trackers; HTC creates generic trackers, named by index at line 292. |
| BD packet reception | `server/driver/wivrn_session.cpp:810` | Obtains existing clock offset, then dispatches into body tracker. |
| BD fan-out | `server/driver/wivrn_body_tracker.cpp:224` | Same input goes to body-joint history and virtual tracker history. This is the first narrow correction hook candidate. |
| BD role mapping | `server/driver/wivrn_body_tracker.cpp:315` | Pelvis → hip, spine3 → chest, elbows/knees/feet → matching virtual roles. |
| Body query | `server/driver/wivrn_body_tracker.cpp:369` | Reads historical/interpolated joints. Patching only here would miss virtual tracker output. |
| HTC reception | `server/driver/wivrn_session.cpp:817` | Directly updates each generic tracker. Needs a separate explicit role mapping adapter. |
| Time conversion | `server/driver/history.h:46`, `server/driver/clock_offset.h:34` | History converts headset production and sample times into host time; do not convert twice. |
| Recenter | `client/scenes/stream_tracking.cpp:421`, `server/driver/wivrn_session.cpp:740` | Recenter flag reaches server; it recenters local spaces. Trace actual space changes before preserving calibration. |
| Prediction | `server/driver/wivrn_generic_tracker.cpp:48` | Tracker history uses velocities or inferred position differences. Position correction with stale valid velocity can create overshoot. |

## Proposed minimal adapter

Use a separate local camera/fusion worker. A WiVRn receiver thread validates
bounded messages and publishes a fixed-size correction snapshot. The
tracking path only reads a snapshot and does bounded per-joint arithmetic.
Never run camera capture, inference, socket reads, JSON parsing, file access,
GUI work, waits or unbounded locks inside tracking callbacks. A local Unix
socket in a user-private runtime directory is a candidate transport; the
framework's development HTTP interface is not the tracking transport.

Start disabled. On BD, apply a correction to a local copy at the fan-out
point so skeleton and virtual trackers agree; keep an untouched baseline
history for camera residual estimation and A/B logging. Never feed fused
poses back as Pico observations. On HTC, require a saved and verified role
map tied to the negotiated session. Unknown route/role means pass-through.
Keep device names and identities stable; do not create duplicate trackers.

The worker should publish **bounded position residuals**, not delayed
absolute poses. Compare camera position at exposure time with interpolated
raw Pico position at that same time. The tracking path adds a fresh,
confidence-weighted residual to the new Pico pose. This preserves Pico's
fast motion while correcting slower drift. It is an initial hypothesis:
measure against direct blending before keeping it. Snapshot metadata must
include protocol version, sequence, session and calibration generation,
host capture time, processing time, source IDs, joint mask, confidence,
validity and expiry. Reject unknown joints, bad dimensions, nonfinite
values, duplicates/out-of-order updates, implausible offsets and future or
expired timestamps before publishing.

Preserve headset/controller poses. Correct body position only initially;
keep Pico orientation and never claim camera-derived roll/twist from an
unobservable joint position. If HTC position changes, either derive
consistent corrected velocity or clear its validity bit so existing
history can derive velocity; test the resulting extrapolation. Preserve
original orientation flags, and do not mark invalid Pico orientation valid
because a camera sees a foot. The BD aggregate `all_tracked` and the current
`is_active = true` implementation are not reliable per-joint confidence.

Camera-only is a separate opt-in path: no incoming body device may exist at
all. The adapter must deliberately register a body/virtual-tracker output set
and populate only valid estimated roles, preserving headset/controller devices.
Never assume the BD receive callback will run without body trackers. On camera
loss, mark body outputs unavailable rather than substituting nonexistent Pico
poses. Device creation and invalidation require their own live integration test.

## Time and space contract

Use host monotonic time throughout the worker. The adapter uses WiVRn's
existing `clock_offset::from_headset` for comparison with raw tracking;
original headset timestamps remain original when submitted to existing
history. Disable assistance until the offset is stable. Timestamp cameras
at exposure when available; an arrival timestamp must be labelled an
estimate with measured buffering uncertainty. Phone streams need their
own clock mapping, not an assumption that wall clocks agree. Never let a
deep capture queue increase latency: consume newest frames, count drops.

Calibrate lens intrinsics, distortion, metric scale and a rigid
camera-to-tracking-space transform for every camera. A headset position
alone cannot recover an arbitrary camera transform and scale. Use visible
tracked targets with known offsets, or a calibrated target plus paired
controller samples at varied non-collinear positions; reserve observations
for validation. Pose-model relative 3D coordinates are not automatically
metric tracking-space coordinates. Reprojection and held-out metric error
must pass before a camera may contribute.

Associate calibration with camera identity, resolution, mounting pose and
tracking-space/session generation. Camera movement, reconnection with
changed configuration, headset session restart and unhandled recenter
invalidate assistance. A recenter may only preserve calibration when the
exact old-to-new transform is known and applied consistently. A stale
snapshot must never be accepted across generations. Flush queued snapshots
on generation changes and on assistance disable.

## Failure behavior and downstream verification

Per-joint confidence combines visibility, geometry, timing uncertainty,
motion consistency and cross-camera agreement. Confidence thresholds need
hysteresis and time-based rise/fall rates. Expired observations contribute
no new correction; a bounded decaying prior can smooth return to Pico.
After its deadline, output must exactly equal current Pico input. Recenter,
bad calibration or protocol violations may require immediate pass-through
because blending across spaces is wrong. If Pico is itself invalid,
preserve invalid flags; camera-only tracking is a later explicit mode.

Validate both BD skeleton consumers and virtual/generic tracker consumers,
including the actual game/runtime combination. An improving skeleton GUI
does not prove the game receives corrected tracker poses. Record callback
duration, p99 and maximum, under worker crash/restart and inference load.
First performance budget: adapter p99 below 0.1 ms on the target host,
no observed callback waits, bounded memory/queues, and no VR frame-time
regression outside baseline measurement variation. These are acceptance
targets, not measured results.
