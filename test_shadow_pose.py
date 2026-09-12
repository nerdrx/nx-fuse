import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    raise unittest.SkipTest('camera extras unavailable')

from shadow_pose import reconstruct


class ShadowPoseTests(unittest.TestCase):
    def setUp(self):
        self.camera = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
        self.model = np.zeros((33, 3), float)
        for i in range(33):
            self.model[i] = ((i % 5 - 2) * .12, (i % 7 - 3) * .13, (i % 3 - 1) * .08)
        self.model[23] = [-.18, .35, 0]; self.model[24] = [.18, .35, 0]
        self.model[11] = [-.28, -.05, .04]; self.model[12] = [.28, -.05, -.03]
        self.model[13] = [-.42, -.28, .08]; self.model[14] = [.42, -.28, -.04]
        self.model[15] = [-.48, -.5, .1]; self.model[16] = [.48, -.5, -.1]
        self.model[25] = [-.16, .8, .02]; self.model[26] = [.16, .8, -.02]
        self.model[27] = [-.16, 1.2, .12]; self.model[28] = [.16, 1.2, -.12]
        self.model[7] = [-.12, -.95, .03]; self.model[8] = [.12, -.95, -.03]
        rvec = np.array([.08, -.12, .03]); tvec = np.array([[0.1], [-.1], [3.4]])
        self.pixels, _ = cv2.projectPoints(self.model, rvec, tvec, self.camera, np.zeros(5))
        self.pixels = self.pixels.reshape(-1, 2)
        self.estimate = {"landmarks": [{"x": float(x / 640), "y": float(y / 480), "visibility": 1.}
                                          for x, y in self.pixels], "inferred3d": self.model.tolist()}
        rot, _ = cv2.Rodrigues(rvec)
        self.alignment = {"camera_to_vr": {"rotation": rot.T.tolist(),
                                            "translation": (-rot.T @ tvec.ravel()).tolist()}}
        ear_cam = (rot @ self.model[[7, 8]].mean(0) + tvec.ravel())
        self.head = {"position": (rot.T @ (ear_cam - tvec.ravel()) + [.05, 0, 0]).tolist(),
                     "orientation": [0, 0, 0, 1], "flags": 0x33}
        self.lens = {"camera_matrix": self.camera.tolist(), "distortion": [0] * 5,
                     "image_size": [640, 480]}

    def test_reconstructs_body_only(self):
        result = reconstruct(self.estimate, self.lens, self.alignment, self.head)
        self.assertLess(result["rms_px"], 1e-3)
        self.assertNotIn("head", result["joints"])
        self.assertNotIn("left_hand", result["joints"])
        self.assertAlmostEqual(result["anchor_shift_m"], .05, places=3)
        self.assertTrue(np.allclose(result["joints"]["hip"]["position"], self.model[[23, 24]].mean(0) + [.05, 0, 0], atol=1e-3))

    def test_missing_confidence_rejected(self):
        self.estimate["landmarks"][11].pop("visibility")
        with self.assertRaises(ValueError):
            reconstruct(self.estimate, self.lens, self.alignment, self.head)

    def test_wrong_head_rejected(self):
        self.head["position"] = [9, 9, 9]
        with self.assertRaises(ValueError):
            reconstruct(self.estimate, self.lens, self.alignment, self.head)

    def test_bad_reprojection_rejected(self):
        self.estimate["landmarks"][13]["x"] += .15
        with self.assertRaises(ValueError):
            reconstruct(self.estimate, self.lens, self.alignment, self.head)

    def test_boolean_and_nan_inputs_rejected(self):
        self.estimate["landmarks"][11]["x"] = True
        with self.assertRaises(ValueError):
            reconstruct(self.estimate, self.lens, self.alignment, self.head)
        self.estimate["landmarks"][11]["x"] = float("nan")
        with self.assertRaises(ValueError):
            reconstruct(self.estimate, self.lens, self.alignment, self.head)


if __name__ == "__main__":
    unittest.main()
