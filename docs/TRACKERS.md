# Tracker-neutral core, Pico-first validation

Pico via WiVRn NX is the first integration target because it is the hardware
available for testing. The core does not import a Pico SDK. `Fusion.step`
accepts a dictionary of named baseline positions, already calibrated into a
common metric space and aligned to camera time. `baseline` is the canonical
replay input key; the early `pico` key remains accepted for existing files.
The demo UI labels its synthetic baseline Pico deliberately.

| Source | Current status | Next proof required |
|---|---|---|
| Synthetic named joints | Implemented and tested | Regression fixtures remain deterministic |
| No body trackers | Camera-only simulation implemented | One-camera 3D inference, calibration, output device creation and loss behavior |
| Pico through WiVRn NX | Planned; source path inspected | Read-only live tap, verified BD/HTC route and roles |
| Other body trackers exposed through WiVRn | Candidate | Verify actual route, roles, timestamps, spaces and valid flags |
| Other IMU trackers, such as SlimeVR | Candidate; no adapter or hardware test | Establish accessible pose output and receiving runtime path |
| Optical trackers, such as Lighthouse systems | Candidate; no adapter or hardware test | Establish pose access, role mapping and evidence camera corrections help |

Do not infer support from the tracker being visible to a game. The integration
must read the original pose and deliver a corrected replacement without
creating duplicate devices. Tracker count and skeletal completeness vary.

Normalize each adapter's output to semantic roles and preserve its original
identity, orientation, validity, capture/sample time and tracking-space
generation outside the position-only fusion core. Missing joints stay missing;
never invent knees from a three-tracker set and label them measured. Distinguish
measured tracker poses from skeleton estimates. Role mapping is explicit and
session-bound, never inferred solely from array order.

Keep camera inference and calibration independent of tracker brands. Only
source acquisition, role/time/space conversion and corrected-pose delivery
belong in the adapter. Protect head and controllers regardless of brand.
Default to pass-through when mapping or calibration is unknown.

Additional adapters must pass synthetic failure/replay tests and then actual
hardware validation for recenter, reconnect, stale input, camera unplugging,
duplicate-device avoidance, correct game output and VR frame-time impact.
Pico results cannot establish another device's support. For an already accurate
optical baseline, measure whether assistance offers any benefit before allowing
camera estimates to override it.

Do not build a plugin loader or bundle unused tracker SDKs yet. Introduce the
next small adapter when its transport and validation hardware are available.
