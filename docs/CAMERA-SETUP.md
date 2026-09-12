# Optional camera setup

NX Fuse simulation stays available with Python's standard library. Camera
capture and MediaPipe pose estimation are optional and never start when the
dashboard opens.

From the `nx-fuse` directory, run:

```sh
./setup-camera.sh
```

This creates or reuses `.venv` and installs the pinned camera dependency from
`requirements-camera.txt`. The script does not download model data by default.

To explicitly download the MediaPipe Pose Landmarker Lite model from Google's
official model storage, run:

```sh
./setup-camera.sh --download-model
```

The downloaded file is written to
`artifacts/models/pose_landmarker_lite.task` only after its SHA-256 checksum
matches the value recorded in the setup script. The existing local artifact can
also be selected directly without downloading:

```sh
NX_FUSE_MODEL="$PWD/artifacts/models/pose_landmarker_lite.task" ./Launch\ NX\ Fuse.sh
```

The native WiVRn NX dashboard discovers cameras and controls capture through
its Body Tracking page. Enable a camera and estimation there explicitly. The
one-camera result is an uncalibrated monocular estimate in model coordinates;
it is not measured camera depth or calibrated headset-space tracking. Camera
mounting, intrinsics, extrinsics, timing, and person association still require
calibration before camera observations should influence a live VR session.
