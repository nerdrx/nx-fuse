import json
import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from spatial_alignment import solve_alignment


@unittest.skipUnless(cv2 is not None and np is not None, "OpenCV and numpy unavailable")
class SpatialAlignmentTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(8)
        self.world = rng.uniform([-1.0, -0.7, 1.5], [1.0, 0.7, 4.0], (16, 3))
        self.camera = np.array([[700., 0., 320.], [0., 710., 240.], [0., 0., 1.]])
        self.distortion = np.array([-0.08, 0.02, 0.001, -0.001, 0.])
        self.rvec = np.array([0.12, -0.18, 0.08])
        self.tvec = np.array([0.15, 0.05, 0.4])
        self.image, _ = cv2.projectPoints(self.world, self.rvec, self.tvec,
                                           self.camera, self.distortion)
        self.image = self.image.reshape(-1, 2)

    def test_known_pose_is_json_safe_and_validated(self):
        report = solve_alignment(self.world, self.image, self.camera, self.distortion, (640, 480))
        json.dumps(report)
        self.assertTrue(report["validated"])
        self.assertLess(report["held_out_rms_px"], 0.01)
        np.testing.assert_allclose(report["camera_to_vr"]["rotation"], cv2.Rodrigues(self.rvec)[0].T, atol=1e-5)
        np.testing.assert_allclose(report["camera_to_vr"]["translation"],
                                   -cv2.Rodrigues(self.rvec)[0].T @ self.tvec, atol=1e-5)

    def test_rejects_coplanar_and_mismatched_samples(self):
        coplanar = self.world.copy()
        coplanar[:, 2] = 2.5
        with self.assertRaises(ValueError):
            solve_alignment(coplanar, self.image, self.camera, self.distortion, (640, 480))
        with self.assertRaises(ValueError):
            solve_alignment(self.world[:-1], self.image, self.camera, self.distortion, (640, 480))

    def test_rejects_behind_camera_solution(self):
        camera_points = np.random.default_rng(4).uniform(
            [-0.7, -0.5, 1.5], [0.7, 0.5, 4.0], (16, 3))
        rotation, _ = cv2.Rodrigues(self.rvec)
        behind_world = (rotation.T @ (-camera_points - self.tvec).T).T
        behind_image, _ = cv2.projectPoints(behind_world, self.rvec, self.tvec,
                                             self.camera, self.distortion)
        with self.assertRaises(ValueError):
            solve_alignment(behind_world, behind_image.reshape(-1, 2),
                            self.camera, self.distortion, (640, 480))

    def test_bad_held_out_clicks_fail_validation(self):
        image = self.image.copy()
        image[-3:] += [22., -18.]
        report = solve_alignment(self.world, image, self.camera, self.distortion, (640, 480))
        self.assertFalse(report["validated"])
        self.assertGreater(report["max_held_out_px"], 6.)


if __name__ == "__main__":
    unittest.main()
