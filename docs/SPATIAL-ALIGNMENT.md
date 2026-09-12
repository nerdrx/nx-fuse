# Align a camera to the headset and controllers

The native camera panel can solve camera-to-VR extrinsics using the headset
alone, either controller alone, or a mixture of all three. One reference must
be moved through at least twelve distinct positions. One stationary headset
point does not constrain full camera orientation and translation.

## Workflow

1. Launch the tracking lab with the updated WiVRn build and connect a headset.
2. Start the selected camera, turn body estimation off, and complete lens setup.
3. Open **Spatial alignment**. Choose Head, Left controller or Right controller.
4. Choose a small, clearly visible physical reference fixed to the device.
   Enter its measured XYZ offset from the tracked head/grip origin, in local
   device coordinates and millimetres. Zero is only an approximation unless
   that visible reference actually coincides with the tracked origin.
5. Hold the device still for at least 400 ms and freeze a frame. Select that
   same reference in the frozen image; numeric pixel controls are also available.
6. Repeat across image width, height and depth. Move at least 3 cm between
   samples; use a much larger volume for a useful fit. Collect at least twelve
   samples, with the final quarter reserved for independent validation.
7. Solve, review training and held-out reprojection error, and export the
   profile. Both sets must meet 3 px RMS and 6 px maximum error to pass.

A controller's tracked grip origin is not its visual centre or the wrist.
Likewise a headset's tracked origin is not its visible front surface. Known
physical offsets matter even when tracking itself is accurate. A fixed visible
mark and measured offset are preferable to an approximate anatomical landmark.
The current workflow selects the reference manually; it does not recognize
headsets/controllers automatically or assume body-model landmarks are devices.

## What the fit means

The solver uses calibrated intrinsics and distortion with OpenCV iterative
[Perspective-n-Point](https://docs.opencv.org/4.10.0/d5/d1f/calib3d_solvePnP.html).
Tracked VR coordinates are matched to image pixels. The exported transform
maps OpenCV camera coordinates (right, down, forward; metres) into the raw
WiVRn tap's tracking space. It is the inverse of OpenCV's VR-to-camera pose.
Noncoplanar training positions, image spread, positive camera depth and
held-out errors are checked. Bad samples are not silently discarded.

This establishes camera extrinsics. It does not establish accurate metric
body depth from a monocular model, wearer association, or a live pose-output
path. The transform is exported for observation/shadow work; no tracker poses
are modified. A low reprojection error alone is not proof of centimetre-level
accuracy: incorrect physical offsets and lens distortion can produce bias.

## Timing, validity and storage

The frame and its host arrival timestamp are captured atomically. WiVRn sample
timestamps are already mapped to the host clock. The selected anchor needs
at least four tracked samples spanning 300 ms, a sample within 60 ms of frame
arrival, and at most 1 cm/2 degrees of motion across the observation window.
Hold still longer when camera latency is unknown; exposure latency has not
been measured by these checks.

Only one frozen JPEG is held in memory, for up to 60 seconds. Accepting a
point discards that JPEG and retains the pixel, pose and offset. Reset removes
the collection. Session/recenter generation, camera restart/resolution, lens
profile changes and loss of fresh tracking invalidate the working collection
and result. Physical movement of a mounted camera cannot be detected reliably;
reset and recalibrate after moving it. Profiles are exported explicitly and
are not automatically restored or applied across sessions.

Synthetic known-transform and workflow checks exercise the solver and state
transitions. No real headset/controller calibration or accuracy measurement
has been performed yet.
