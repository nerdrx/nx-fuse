import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from estimator import LANDMARK_NAMES, PoseEstimator


def fake_landmarks(offset=0):
    return [SimpleNamespace(x=i / 100 + offset, y=.5, z=i / 100, visibility=.9)
            for i in range(len(LANDMARK_NAMES))]


def fake_world_landmarks():
    return [SimpleNamespace(x=i / 1000, y=i / 2000, z=i / 3000)
            for i in range(len(LANDMARK_NAMES))]


class FakeLandmarker:
    def __init__(self, poses):
        self.poses = poses
        self.calls = 0
        self.closed = False

    def detect(self, frame):
        self.calls += 1
        return SimpleNamespace(pose_landmarks=self.poses,
                               pose_world_landmarks=[fake_world_landmarks()])

    def close(self):
        self.closed = True


class EstimatorTests(unittest.TestCase):
    def model(self):
        handle = tempfile.NamedTemporaryFile(suffix='.task', delete=False)
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def test_lazy_fake_estimate_and_hip_relative_output(self):
        fake = FakeLandmarker([fake_landmarks()])
        made = []
        estimator = PoseEstimator(self.model(), landmarker_factory=lambda path: (made.append(path), fake)[1])
        self.assertEqual(made, [])
        output = estimator.estimate(np.zeros((4, 5, 3), dtype=np.uint8))
        self.assertEqual(len(made), 1)
        self.assertEqual(output.landmarks[0]['name'], 'nose')
        self.assertAlmostEqual(output.inferred3d[23][0], -.0005)
        self.assertAlmostEqual(output.inferred3d[23][2], -.0001667)
        estimator.close()
        self.assertTrue(fake.closed)

    def test_rejects_ambiguous_or_invalid_pose(self):
        fake = FakeLandmarker([fake_landmarks(), fake_landmarks(.1)])
        estimator = PoseEstimator(self.model(), landmarker_factory=lambda _: fake)
        self.assertIsNone(estimator.estimate(np.zeros((2, 2, 3), dtype=np.uint8)))
        with self.assertRaises(ValueError):
            estimator.estimate(np.zeros((2, 2), dtype=np.uint8))

    def test_rejects_malformed_world_landmarks_and_visibility(self):
        fake = FakeLandmarker([fake_landmarks()])
        fake.poses[0][0].visibility = 2
        estimator = PoseEstimator(self.model(), landmarker_factory=lambda _: fake)
        self.assertIsNone(estimator.estimate(np.zeros((2, 2, 3), dtype=np.uint8)))
        fake.poses[0][0].visibility = .9
        original = fake_world_landmarks
        globals()['fake_world_landmarks'] = lambda: [SimpleNamespace(x=0, y=0, z=float('nan'))] * len(LANDMARK_NAMES)
        try:
            self.assertIsNone(estimator.estimate(np.zeros((2, 2, 3), dtype=np.uint8)))
        finally:
            globals()['fake_world_landmarks'] = original


if __name__ == '__main__':
    unittest.main()
