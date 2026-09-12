import unittest

try:
    import numpy as np
except ImportError:
    np = None

from calibration import _payload, fit_rigid


@unittest.skipUnless(np is not None, 'numpy unavailable')
class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.model = np.array([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [1, 2, 3]], float)
        self.rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
        self.translation = np.array([2, -1, .5])
        self.target = self.model @ self.rotation.T + self.translation

    def test_known_rigid_transform_and_held_out_errors(self):
        report = fit_rigid(self.model, self.target, self.model[:1], self.target[:1])
        np.testing.assert_allclose(report.transform.apply(self.model), self.target, atol=1e-12)
        self.assertLess(report.rms_error, 1e-12)
        self.assertEqual(len(report.held_out_errors), 1)

    def test_small_noise_reports_fit_and_validation_errors(self):
        noisy = self.target.copy()
        noisy[0] += [.01, 0, 0]
        report = fit_rigid(self.model, noisy)
        self.assertGreater(report.rms_error, 0)
        self.assertGreater(report.max_error, report.rms_error)

    def test_rejects_degenerate_reflection_nonfinite_and_wrong_units(self):
        with self.assertRaises(ValueError):
            fit_rigid(self.model[:4, :2].tolist(), self.target[:4])
        with self.assertRaises(ValueError):
            fit_rigid(self.model, self.target * [1, 1, -1])
        bad = self.target.copy(); bad[0, 0] = np.nan
        with self.assertRaises(ValueError):
            fit_rigid(self.model, bad)
        with self.assertRaises(ValueError):
            fit_rigid(self.model, self.target * 100)

    def test_rejects_boolean_points_and_invalid_limits(self):
        with self.assertRaises(ValueError):
            fit_rigid(self.model.tolist(), self.target.tolist(), max_error=float('nan'))
        with self.assertRaises(ValueError):
            fit_rigid([[True, 0, 0], *self.model[1:].tolist()], self.target)

    def test_cli_metadata_requires_explicit_valid_fields_and_holdout(self):
        valid = {'camera_id': 'cam', 'resolution': [640, 480], 'session_generation': 1,
                 'fit': [{'model': [0, 0, 0], 'target': [0, 0, 0]}] * 4,
                 'validation': [{'model': [0, 0, 0], 'target': [0, 0, 0]}]}
        self.assertIs(_payload(valid), valid)
        for field, value in [('camera_id', ''), ('resolution', [640, True]),
                             ('session_generation', -1), ('validation', [])]:
            bad = dict(valid); bad[field] = value
            with self.assertRaises(ValueError):
                _payload(bad)
        bad = dict(valid); bad['extra'] = 1
        with self.assertRaises(ValueError):
            _payload(bad)
