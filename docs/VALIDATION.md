# Framework validation

Performed 2026-09-12 on the development Linux host.

- `python3 -m unittest -v`: ten tests pass. Includes camera-only output and
  unavailable joints without a body baseline, plus gradual camera-loss
  release, hard expiry, protected head, invalid/stale/future observations,
  conflicting views, repeated-source neutrality, long process gaps, reset,
  deterministic replay, malformed frames and HTTP control validation.
- Chrome inside a headless gamescope compositor: simulator connects;
  assistance changes source display; occlusion returns to Pico fallback;
  camera-only loss marks the body unavailable; side-view selection works;
  debug observations distinguish observed/missing inputs; no JavaScript page errors.
- Console and website checked at 1440-pixel desktop and 390-pixel mobile
  widths, with no horizontal document overflow; screenshots visually inspected.
- Linux video-device names were enumerated from sysfs. No capture device
  was opened. Multiple nodes can belong to one physical camera.

No live Pico session, camera inference, calibration, headset output, motion
accuracy, real-time performance or physical multi-camera coverage was tested.
The Python HTTP service is a local development console, not a VR driver.
