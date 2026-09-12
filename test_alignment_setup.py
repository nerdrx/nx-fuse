import json
import unittest
from unittest.mock import patch

from alignment_setup import AlignmentSetup

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


class Camera:
    epoch = 'one'
    def status(self, camera):
        return dict(streaming=True, frame_stale=False, image_size=[640,480], epoch=self.epoch)
    def frame_snapshot(self, camera):
        return dict(jpeg=b'fixture', time_ns=1, epoch=self.epoch, image_size=[640,480])


class Lens:
    profile = dict(validated=True, camera_id='camera', image_size=[640,480],
                   camera_matrix=[[500,0,320],[0,500,240],[0,0,1]], distortion=[0]*5)
    def snapshot(self):
        return {'profile':self.profile}


class Tracking:
    generation = '1'
    connected = True
    position = [0,0,2]
    def snapshot(self):
        return dict(connected=self.connected, generation=self.generation)
    def stable_anchor(self, anchor, timestamp, offset):
        return dict(position=self.position, generation=self.generation, match_error_ms=2)


class AlignmentWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.camera, self.lens, self.tracking = Camera(), Lens(), Tracking()
        self.setup = AlignmentSetup(self.camera, self.lens, self.tracking)
    def freeze(self):
        self.setup.command(dict(action='freeze', id='camera', anchor='head', offset=[0,0,0]))
        return self.setup.snapshot()['pending']['token']
    def sample(self, token, pixel):
        self.setup.command(dict(action='sample',id='camera',token=token,pixel=pixel))
    def test_tokens_pixels_expiry_and_recenter(self):
        token = self.freeze()
        self.assertEqual(self.setup.frame(token), b'fixture')
        self.assertIsNone(self.setup.frame('wrong'))
        for pixel in ([640,20], [True,20], [float('nan'),20]):
            with self.assertRaises(ValueError): self.sample(token,pixel)
        self.sample(token,[320,240])
        with self.assertRaises(ValueError): self.sample(token,[320,240])
        token = self.freeze()
        with self.assertRaisesRegex(ValueError,'Move'): self.sample(token,[320,240])
        self.tracking.generation = '2'
        state = self.setup.snapshot()
        self.assertEqual(state['count'],0)
        self.assertIsNone(state['pending'])
        self.assertIn('changed',state['error'])
        token = self.freeze()
        with patch('alignment_setup.time.monotonic', return_value=float('inf')):
            self.assertIsNone(self.setup.frame(token))
    def test_restart_and_tracking_loss_invalidate(self):
        self.freeze()
        self.camera.epoch = 'two'
        self.assertIsNone(self.setup.snapshot()['pending'])
        self.freeze()
        self.tracking.connected = False
        self.assertIsNone(self.setup.snapshot()['pending'])
    @unittest.skipIf(np is None, 'OpenCV extras unavailable')
    def test_collection_solves_known_camera_and_profile_invalidates(self):
        rng = np.random.default_rng(72)
        # Diverse VR anchor positions, camera rotation and translation known independently.
        points = rng.uniform([-.7,-.5,2], [.7,.5,3], (16,3))
        rvec = np.array([.1,-.15,.05]); tvec=np.array([.1,.05,.3])
        pixels,_ = cv2.projectPoints(points,rvec,tvec,np.array(self.lens.profile['camera_matrix'],float),np.zeros(5))
        for position,pixel in zip(points,pixels.reshape(-1,2)):
            self.tracking.position=position.tolist()
            self.sample(self.freeze(),pixel.tolist())
        self.setup.command(dict(action='solve',id='camera'))
        profile = self.setup.snapshot()['profile']
        self.assertTrue(profile['validated'])
        rotation,_=cv2.Rodrigues(rvec)
        np.testing.assert_allclose(profile['camera_to_vr']['translation'],-rotation.T@tvec,atol=1e-6)
        json.dumps(profile,allow_nan=False)
        self.lens.profile=dict(self.lens.profile, distortion=[.01,0,0,0,0])
        self.assertIsNone(self.setup.snapshot()['profile'])

    @unittest.skipIf(np is None, 'OpenCV extras unavailable')
    def test_real_receiver_capture_and_http_routes(self):
        import tempfile
        import time
        import threading
        import urllib.request
        from pathlib import Path
        from http.server import ThreadingHTTPServer
        from tracking import TrackingReceiver, HEADER, RECORD, decode_packet
        from app import Simulation, make_handler
        with tempfile.TemporaryDirectory() as directory:
            receiver = TrackingReceiver(Path(directory)/'tap.sock')
            now = time.monotonic_ns()
            try:
                for sequence, delta in enumerate((300,200,100,0),1):
                    stamp=now-delta*1_000_000
                    wire=HEADER.pack(b'NXTP',1,40,sequence,stamp,1,0,1,1,0)+RECORD.pack(1,0,0x33,0,0,2,0,0,0,1,0,0,0,0,0,0,stamp)
                    receiver._accept(decode_packet(wire,now))
                sim=Simulation(camera_manager=self.camera,tracking_receiver=receiver)
                sim.lens_setup=self.lens
                sim.alignment_setup=AlignmentSetup(self.camera,self.lens,receiver)
                self.camera.frame_snapshot=lambda camera: dict(jpeg=b'fixture',time_ns=now,epoch='one',image_size=[640,480])
                server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(sim))
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                http=urllib.request.build_opener(urllib.request.ProxyHandler({}))
                base=f'http://127.0.0.1:{server.server_port}'
                def post(value):
                    with http.open(urllib.request.Request(base+'/api/alignment',data=json.dumps(value).encode(),headers={'Content-Type':'application/json'})) as response:
                        return json.load(response)
                try:
                    self.assertEqual(post(dict(action='freeze',id='camera',anchor='head',offset=[0,0,0])),{'ok':True})
                    with http.open(base+'/api/alignment') as response: state=json.load(response)
                    token=state['pending']['token']
                    with http.open(base+'/api/alignment/frame?token='+token) as response: self.assertEqual(response.read(),b'fixture')
                    post(dict(action='sample',id='camera',token=token,pixel=[320,240]))
                    self.assertEqual(sim.alignment_setup.snapshot()['count'],1)
                    post(dict(action='reset'))
                    self.assertEqual(sim.alignment_setup.snapshot()['count'],0)
                finally:
                    server.shutdown();server.server_close();thread.join()
            finally: receiver.close()


if __name__ == '__main__': unittest.main()
