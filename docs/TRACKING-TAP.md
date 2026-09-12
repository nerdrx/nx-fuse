# Read-only WiVRn tracking lab

The native dashboard can inspect raw headset/controller and BD/HTC tracker
observations through a private local Unix datagram socket. This does not
correct poses, inject trackers, align cameras, or feed the simulation.

## Launch

Build the updated WiVRn NX server and dashboard, then close any existing
WiVRn server/session so the dashboard cannot attach to an older server.
From the NX Fuse checkout:

```sh
.venv/bin/python tracking_lab.py --dashboard /path/to/wivrn-dashboard --port 8788
```

The launcher starts the worker first, then gives the dashboard and its child
server `NX_FUSE_TAP=$XDG_RUNTIME_DIR/nx-fuse-8788/tracking.sock`. Open **Fuse →
Inspect raw poses** after connecting a headset. Cameras remain off until
explicitly started. `--preview` disables VR server startup/attachment for
UI testing; it cannot receive headset poses by itself.

Closing the dashboard stops only the worker owned by that launcher. An
already-running compatible worker is reused and left running. An occupied
port in another mode produces an error. Existing socket files are never
silently replaced. After a hard crash, first establish that no worker owns
the path before removing a stale socket or choose a different port.

The server connects to the socket once at session construction. Start the
worker before the headset session. Restart that session after restarting
the worker. A missing socket or unstable clock offset produces no packets.

## Transport version 1

All integers and IEEE-754 floats are little endian. Each datagram contains
one 40-byte header followed by at most 32 records of 68 bytes each.

| Header | Type |
|---|---|
| Magic `NXTP`, version 1, header size 40 | 4 bytes, u16, u16 |
| Sequence, host monotonic send time in ns, session/space generation | u64 each |
| Route, record count, flags, reserved zero | u8, u8, u16, u32 |

Route 0 means anchors, 1 BD body, 2 HTC body. Header flag bit 0 means the
clock offset is stable; other bits are reserved. Each record contains role
u16, original source index u16, validity flags u32, position xyz f32,
orientation xyzw f32, linear velocity xyz f32, angular velocity xyz f32,
and sample time u64 mapped to host monotonic nanoseconds. Sample time may
be predicted ahead of reception; send time is not camera exposure time.

Validity bits 0–5 mean position valid, orientation valid, linear velocity
valid, angular velocity valid, position tracked, orientation tracked.
Roles 1–7 are head, left/right grip, left/right aim, left/right palm.
Roles 10–17 are hip, chest, left/right elbow, left/right knee, left/right
foot. Unmapped BD joints use `0x0100 | index`; generic HTC trackers use
`0x8000 | index`. Generic indices do not establish anatomical body roles.
META body packets are not supported by this tap.

The tap uses bounded stack storage and nonblocking datagram sends. Full or
unavailable receivers drop observations; the normal WiVRn update continues.
The socket and its parent must belong to the same user and be private
(socket 0600, directory 0700). The receiver rejects malformed, nonfinite,
stale, reordered and retired-generation packets. Visible records expire
after 500 ms. A session has an independent generation; recenter increments
it without changing WiVRn role generation. These identifiers are observation
metadata, not a saved camera calibration.

## Validation and limits

The C++ serializer has a cross-language Unix-socket fixture decoded by the
actual Python receiver parser:

```sh
python3 tests/run_nx_fuse_tap_test.py --nx-fuse-root /path/to/nx-fuse
```

Run that command from WiVRn NX after configuring `build-server` with compile
commands. The fixture links the production serializer, with conversion and
clock stubs; it does not validate the live clock estimator. Python tests
also exercise expiry, sequencing, generations and socket ownership.
Native headless inspection was tested with explicitly synthetic packets.
No real Pico session, VR timing regression measurement, camera-to-VR
alignment, or tracking improvement has been demonstrated by these checks.
