import math
import unittest

from shadow_report import NOTE, build_report


def frame(timestamp, distance=1.0, baseline=True, camera='cam', generation='g'):
    return {'camera': camera, 'generation': generation, 'timestamp_ns': timestamp,
            'joints': [{'joint': 'head', 'position': [distance, 0, 0], 'confidence': 1,
                        'baseline': [0, 0, 0] if baseline else None,
                        'residual_m': 999}],
            'metrics': {'rms_px': distance, 'anchor_shift_m': distance / 10,
                        'match_error_ms': 2}, 'note': 'model'}


class ShadowReportTests(unittest.TestCase):
    def test_known_distance_stats_and_recomputed_residual(self):
        report = build_report({'format': 'nx-fuse-shadow-v1',
                               'frames': [frame(1, 1), frame(2, 3)]})
        group = report['groups'][0]
        self.assertEqual(group['frames'], 2)
        self.assertEqual(group['joints']['head']['baseline_paired_count'], 2)
        residual = group['joints']['head']['residual_m']
        self.assertEqual(residual['median'], 2)
        self.assertAlmostEqual(residual['rms'], math.sqrt(5))
        self.assertEqual(report['note'], NOTE)

    def test_zero_frames_and_unpaired_baseline_are_null(self):
        empty = build_report({'format': 'nx-fuse-shadow-v1', 'frames': []})
        self.assertEqual(empty['groups'], [])
        report = build_report({'format': 'nx-fuse-shadow-v1', 'frames': [frame(1, baseline=False)]})
        self.assertIsNone(report['groups'][0]['joints']['head']['residual_m'])

    def test_rejects_nan_and_timestamp_order(self):
        bad = frame(1)
        bad['metrics']['rms_px'] = float('nan')
        with self.assertRaises(ValueError):
            build_report({'format': 'nx-fuse-shadow-v1', 'frames': [bad]})
        with self.assertRaises(ValueError):
            build_report({'format': 'nx-fuse-shadow-v1', 'frames': [frame(2), frame(1)]})


if __name__ == '__main__':
    unittest.main()
