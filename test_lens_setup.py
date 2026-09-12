import time
import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from lens_setup import LensSetup


class LensSetupTests(unittest.TestCase):
    def profile(self):
        return {'camera_id': 'saved', 'image_size': [640, 480],
                'camera_matrix': [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
                'distortion': [0, 0, 0, 0, 0], 'rms_error': .2,
                'train_errors': [.2] * 5, 'held_out_errors': [.5],
                'train_indices': [0, 1, 2, 3, 4], 'held_out_indices': [5],
                'validated': True, 'held_out_rms_px': 99,
                'scope': 'ignored'}

    def test_load_profile_roundtrip_and_recomputes_validation(self):
        setup = LensSetup()
        setup.load_profile(self.profile())
        snapshot = setup.snapshot()
        self.assertEqual(snapshot['camera'], 'saved')
        self.assertEqual(snapshot['count'], 0)
        self.assertTrue(snapshot['profile']['validated'])
        self.assertLess(snapshot['profile']['held_out_rms_px'], 1)
        self.assertEqual(snapshot['profile']['scope'], 'lens intrinsics only; not camera-to-VR calibration')

    def test_load_profile_rejects_bad_geometry_and_metrics(self):
        for key, value in [('camera_matrix', [[1, 0, 0], [0, 1, 0], [1, 0, 0]]),
                           ('distortion', ['0'] * 5), ('rms_error', float('nan'))]:
            profile = self.profile(); profile[key] = value
            with self.assertRaises(ValueError): LensSetup().load_profile(profile)
        profile = self.profile(); profile['held_out_errors'] = [3]
        loaded = LensSetup(); loaded.load_profile(profile)
        self.assertFalse(loaded.snapshot()['profile']['validated'])

    def test_load_profile_busy_rejected(self):
        setup = LensSetup(); setup._busy = True
        with self.assertRaises(ValueError): setup.load_profile(self.profile())

    @unittest.skipIf(cv2 is None, 'optional camera packages unavailable')
    def test_explicit_collection_solve_and_reset(self):
        setup = LensSetup()
        class Camera:
            jpeg = None
            def status(self, _): return {'streaming': True, 'estimation': False}
            def frame(self, _): return self.jpeg
        camera = Camera()
        board = np.full((210, 300), 255, np.uint8)
        for row in range(7):
            for col in range(10):
                if (row+col)%2 == 0: board[row*30:(row+1)*30, col*30:(col+1)*30] = 0
        corners = np.float32([[0,0,0],[.25,0,0],[.25,.175,0],[0,.175,0]])
        source = np.float32([[0,0],[299,0],[299,209],[0,209]])
        matrix = np.float64([[620,0,320],[0,620,240],[0,0,1]])
        def wait():
            deadline=time.monotonic()+5
            while setup.snapshot()['busy'] and time.monotonic()<deadline: time.sleep(.005)
            self.assertFalse(setup.snapshot()['busy'])
            self.assertEqual(setup.snapshot()['error'], '')
        for index in range(9):
            rotation = np.float64([-.25+.06*index, .18-.04*index, -.15+.035*index])
            translation = np.float64([-.15+.008*index, -.12+.009*index, .65+.035*(index%4)])
            target, _ = cv2.projectPoints(corners, rotation, translation, matrix, np.zeros(5))
            transform = cv2.getPerspectiveTransform(source, target.reshape(4,2).astype(np.float32))
            frame = cv2.warpPerspective(board, transform, (640,480), borderValue=255)
            camera.jpeg = cv2.imencode('.jpg',frame)[1].tobytes()
            setup.command({'action':'capture','id':'fixture'}, camera)
            wait()
        self.assertEqual(setup.snapshot()['count'], 9)
        with self.assertRaises(ValueError): setup.command({'action':'solve','id':'other'},camera)
        setup.command({'action':'solve','id':'fixture'}, camera)
        wait()
        self.assertTrue(setup.snapshot()['profile']['validated'])
        self.assertLess(setup.snapshot()['profile']['held_out_rms_px'], 1)
        setup.command({'action':'reset'},camera)
        self.assertEqual(setup.snapshot()['count'], 0)
        self.assertIsNone(setup.snapshot()['profile'])


if __name__ == '__main__': unittest.main()
