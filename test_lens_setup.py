import time
import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from lens_setup import LensSetup


@unittest.skipIf(cv2 is None, 'optional camera packages unavailable')
class LensSetupTests(unittest.TestCase):
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
