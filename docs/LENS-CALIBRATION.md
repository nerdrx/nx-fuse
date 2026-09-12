# Offline lens calibration

`lens_calibration.py` fits camera intrinsics and radial/tangential distortion
from chessboard photographs. It does not estimate camera pose, camera-to-VR
extrinsics, or metric body tracking.

Use at least 6 sharp views of one printed board; 8–20 gives more useful
validation. Keep the full board visible and vary position, distance, and tilt.
Every image must have the same resolution. The board must have the stated
number of **inner corners** and a measured square size. Measure the printed
square with a ruler; pass that size in metres.

Generate a board SVG when needed:

```sh
./.venv/bin/python lens_calibration.py --cols 9 --rows 6 \
  --square-size 0.025 --board-svg board.svg
```

Print in landscape at 100% scale. Disable “fit to page” and verify one printed
square measures 25 mm (or the requested size). The default 9×6 board is 270×195
mm and fits A4 landscape with 10 mm margins. A board with 9×6 inner corners has
10×7 black/white squares. The SVG label records the intended dimensions.

Native capture workflow:

1. Select the active camera in the native camera view. Turn **estimation off**;
   lens calibration needs the image only and must not feed body estimates.
2. Capture at least 6 accepted board views. Move the board through the frame,
   change distance, and add moderate tilt. Keep resolution unchanged.
3. Pass captured frames to `collect_corners`, or call `detect_corners` for each
   frame and retain only successful detections. Do not store camera images in a
   lens profile.
4. Call `calibrate_lens`. Accept a profile only when held-out RMS reprojection
   error is below 1.0 px and no held-out view exceeds 2.0 px. These are setup
   gates, not a claim of tracking accuracy.
5. Export the JSON result. It contains intrinsics and distortion only; camera
   pose, camera-to-VR extrinsics, and body tracking remain separate steps.

Calibrate image files directly:

```sh
./.venv/bin/python lens_calibration.py --cols 9 --rows 6 \
  --square-size 0.025 'captures/*.png' > lens-profile.json
```

The JSON output includes camera matrix, distortion coefficients, training
reprojection error, and held-out reprojection error in pixels. Held-out error
comes from solving each withheld board pose using the fitted intrinsics. Treat
the result as an offline lens profile; it is not proof of real-world tracking
accuracy.

Python callers use `detect_corners(frame, (cols, rows), square_size)` for one
GUI frame, or `collect_corners(frames, (cols, rows), square_size)` for a batch,
then call `calibrate_lens(dataset)`. `square_size` is metres. The calibrator rejects too
few views, inconsistent dimensions, non-finite data, and views lacking useful
position/scale diversity.

## Restore a native profile

Use **Load lens profile…** to select a JSON file exported by the native panel.
The file stays local; geometry, dimensions, errors and view indices are checked.
The imported validation flag is ignored and the held-out gate is recomputed.
Loading does not start cameras or restore spatial alignment. Keep the same
camera/focus/zoom/resolution, or calibrate the lens again.
