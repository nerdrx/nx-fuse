import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from lens_calibration import ChessboardCorners, calibration_board_svg, calibrate_lens, collect_corners


@unittest.skipUnless(cv2 is not None and np is not None, 'OpenCV and numpy unavailable')
class LensCalibrationTests(unittest.TestCase):
    def test_synthetic_known_lens_has_low_held_out_reprojection(self):
        pattern = (9, 6)
        grid = np.zeros((54, 3), np.float32)
        grid[:, :2] = np.indices((6, 9)).transpose(1, 2, 0).reshape(-1, 2) * 0.04
        matrix = np.array([[700., 0, 320], [0, 710., 240], [0, 0, 1]], np.float64)
        distortion = np.array([-0.12, 0.03, 0.001, -0.002, 0.0])
        objects, images = [], []
        for i in range(12):
            rvec = np.array([0.04 * np.sin(i), -0.08 + i * 0.012, 0.02 * np.cos(i)])
            tvec = np.array([-0.12 + i * 0.018, -0.08 + (i % 3) * 0.04, 2.4 + (i % 4) * 0.18])
            points, _ = cv2.projectPoints(grid, rvec, tvec, matrix, distortion)
            objects.append(grid.copy())
            images.append(points.astype(np.float32))
        dataset = ChessboardCorners(pattern, 0.04, (640, 480), tuple(objects), tuple(images), tuple(range(12)))
        report = calibrate_lens(dataset)
        self.assertLess(report.held_out_rms, 0.2)
        np.testing.assert_allclose(np.asarray(report.camera_matrix), matrix, rtol=0.03, atol=8)

    def test_rejects_inconsistent_frames_and_degenerate_views(self):
        with self.assertRaises(ValueError):
            collect_corners([np.zeros((100, 100), np.uint8), np.zeros((101, 100), np.uint8)], (3, 3), 0.03)
        points = tuple(np.zeros((9, 1, 2), np.float32) for _ in range(6))
        objects = tuple(np.zeros((9, 3), np.float32) for _ in range(6))
        dataset = ChessboardCorners((3, 3), 0.03, (100, 100), objects, points, tuple(range(6)))
        with self.assertRaises(ValueError):
            calibrate_lens(dataset)

    def test_board_svg_states_dimensions_and_print_guidance(self):
        svg = calibration_board_svg((9, 6), 25)
        self.assertIn('width="270mm"', svg)
        self.assertIn('print at 100%', svg)


if __name__ == '__main__':
    unittest.main()
