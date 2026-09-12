import http.client
import json
import os
import threading
import time
import unittest
from http.server import ThreadingHTTPServer

from app import Simulation, make_handler
from camera import CameraManager


class FakeCapture:
    def __init__(self): self.closed = False; self.count = 0
    def isOpened(self): return True
    def read(self):
        if self.closed: return False, None
        self.count += 1
        time.sleep(.001)
        return True, self.count
    def release(self): self.closed = True


class FailingCapture(FakeCapture):
    def read(self):
        if self.closed: return False, None
        if self.count:
            return False, None
        self.count += 1
        return True, 1


class FakeEstimate:
    landmarks = tuple({'name': 'hip', 'x': .5, 'y': .5} for _ in range(40))
    inferred3d = tuple((0., 0., 0.) for _ in range(40))
    coordinate_note = 'fake camera coordinates'
    def close(self): pass


class FakeEstimator:
    def estimate(self, _frame): return FakeEstimate()
    def close(self): pass


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.capture = FakeCapture()
        self.manager = CameraManager(
            devices=lambda: [{'id':'cam-a', 'name':'Fake room'}, {'id':'cam-b', 'name':'Fake bed'}],
            capture_factory=lambda _: self.capture,
            jpeg_encoder=lambda frame: b'jpeg:' + str(frame).encode())

    def tearDown(self): self.manager.close()

    def test_discovery_does_not_open_and_capture_is_explicit(self):
        self.assertEqual([d['id'] for d in self.manager.devices()], ['cam-a', 'cam-b'])
        self.assertEqual(self.capture.count, 0)
        self.manager.set_enabled('cam-a', True)
        for _ in range(100):
            if self.manager.frame('cam-a'): break
            time.sleep(.002)
        self.assertTrue(self.manager.frame('cam-a').startswith(b'jpeg:'))
        self.assertTrue(self.manager.status('cam-a')['streaming'])
        self.manager.stop('cam-a')
        self.assertFalse(self.manager.status('cam-a')['streaming'])
        self.assertTrue(self.capture.closed)

    def test_http_control_and_latest_frame(self):
        sim = Simulation(self.manager); sim.tick()
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(sim))
        thread = threading.Thread(target=server.serve_forever); thread.start()
        client = http.client.HTTPConnection('127.0.0.1', server.server_port)
        try:
            client.request('GET', '/api/cameras')
            self.assertEqual(len(json.loads(client.getresponse().read())['devices']), 2)
            client.request('POST', '/api/camera/control', json.dumps({'id':'cam-a','enabled':True}))
            self.assertEqual(client.getresponse().status, 200)
            for _ in range(100):
                client.request('GET', '/api/camera/frame?id=cam-a')
                response = client.getresponse(); data = response.read()
                if response.status == 200:
                    self.assertLessEqual(len(data), 2_000_000)
                    self.assertTrue(data.startswith(b'jpeg:')); break
                time.sleep(.002)
            else: self.fail('fake capture did not produce frame')
            client.request('POST', '/api/camera/control', json.dumps({'id':'cam-a','enabled':False}))
            self.assertEqual(client.getresponse().status, 200)
        finally:
            client.close(); server.shutdown(); server.server_close(); thread.join()

    def test_duplicate_start_and_restart_after_stop(self):
        captures = []
        def factory(_):
            capture = FakeCapture(); captures.append(capture); return capture
        manager = CameraManager(
            devices=lambda: [{'id':'cam-a', 'name':'Fake room'}],
            capture_factory=factory,
            jpeg_encoder=lambda frame: b'jpeg')
        try:
            first = manager.set_enabled('cam-a', True)
            again = manager.set_enabled('cam-a', True)
            self.assertEqual(len(captures), 1)
            self.assertEqual(first['id'], again['id'])
            manager.stop('cam-a')
            manager.set_enabled('cam-a', True)
            self.assertEqual(len(captures), 2)
        finally:
            manager.close()

    def test_disappeared_device_remains_stoppable(self):
        devices = [{'id': 'cam-a', 'name': 'Fake room'}]
        self.manager._devices = lambda: list(devices)
        self.manager.set_enabled('cam-a', True)
        devices.clear()
        self.assertEqual(self.manager.devices()[0]['id'], 'cam-a')
        self.manager.set_enabled('cam-a', False)
        self.assertEqual(self.manager.devices(), [])
        self.assertTrue(self.capture.closed)

    def test_stalled_capture_hides_frame_without_losing_stop_control(self):
        gate = threading.Event()
        class StalledCapture(FakeCapture):
            def read(self):
                if self.count:
                    gate.wait(2)
                return super().read()
        manager = CameraManager(devices=lambda: [{'id':'cam-a','name':'Fake'}],
                                capture_factory=lambda _: StalledCapture(),
                                jpeg_encoder=lambda _: b'jpeg')
        try:
            manager.set_enabled('cam-a', True)
            for _ in range(100):
                if manager.frame('cam-a'): break
                time.sleep(.002)
            with manager._lock:
                manager._streams['cam-a'].last_frame -= 3
            self.assertTrue(manager.status('cam-a')['streaming'])
            self.assertTrue(manager.status('cam-a')['frame_stale'])
            self.assertIsNone(manager.frame('cam-a'))
        finally:
            gate.set()
            manager.close()

    def test_failed_stream_hides_stale_frame_and_can_restart(self):
        captures = [FailingCapture(), FakeCapture()]
        manager = CameraManager(
            devices=lambda: [{'id':'cam-a', 'name':'Fake room'}],
            capture_factory=lambda _: captures.pop(0),
            jpeg_encoder=lambda frame: b'jpeg')
        try:
            manager.set_enabled('cam-a', True)
            for _ in range(100):
                if manager.status('cam-a')['error']: break
                time.sleep(.002)
            self.assertFalse(manager.status('cam-a')['streaming'])
            self.assertIsNone(manager.frame('cam-a'))
            manager.set_enabled('cam-a', True)
            self.assertTrue(manager.status('cam-a')['streaming'])
        finally:
            manager.close()

    def test_opt_in_estimation_is_bounded_and_clears(self):
        manager = CameraManager(
            devices=lambda: [{'id':'cam-a', 'name':'Fake room'}],
            capture_factory=lambda _: FakeCapture(),
            jpeg_encoder=lambda frame: b'jpeg',
            estimator_factory=lambda _: FakeEstimator())
        old_model = os.environ.get('NX_FUSE_MODEL')
        os.environ['NX_FUSE_MODEL'] = '/explicit/model.task'
        try:
            manager.set_enabled('cam-a', True)
            manager.set_estimation('cam-a', True)
            for _ in range(100):
                if manager.status('cam-a')['estimate']: break
                time.sleep(.002)
            estimate = manager.status('cam-a')['estimate']
            self.assertEqual(len(estimate['landmarks']), 33)
            self.assertEqual(manager.status('cam-a')['estimate_sequence'] > 0, True)
            manager.set_estimation('cam-a', False)
            self.assertIsNone(manager.status('cam-a')['estimate'])
        finally:
            if old_model is None: os.environ.pop('NX_FUSE_MODEL', None)
            else: os.environ['NX_FUSE_MODEL'] = old_model
            manager.close()

    def test_disable_during_inference_drops_result_and_closes_on_worker(self):
        entered, finish, closed = threading.Event(), threading.Event(), threading.Event()
        owner = []
        class SlowEstimator:
            def estimate(self, _frame):
                owner.append(threading.get_ident())
                entered.set()
                finish.wait(2)
                return FakeEstimate()
            def close(self):
                self_thread = threading.get_ident()
                if owner and self_thread == owner[0]:
                    closed.set()
        manager = CameraManager(
            devices=lambda: [{'id':'cam-a', 'name':'Fake room'}],
            capture_factory=lambda _: FakeCapture(),
            jpeg_encoder=lambda frame: b'jpeg',
            estimator_factory=lambda _: SlowEstimator())
        old_model = os.environ.get('NX_FUSE_MODEL')
        os.environ['NX_FUSE_MODEL'] = '/explicit/model.task'
        try:
            manager.set_enabled('cam-a', True)
            manager.set_estimation('cam-a', True)
            self.assertTrue(entered.wait(1))
            manager.set_estimation('cam-a', False)
            self.assertFalse(closed.is_set())
            self.assertIsNone(manager.frame('cam-a'))
            finish.set()
            self.assertTrue(closed.wait(1))
            self.assertIsNone(manager.status('cam-a')['estimate'])
        finally:
            finish.set()
            manager.close()
            if old_model is None: os.environ.pop('NX_FUSE_MODEL', None)
            else: os.environ['NX_FUSE_MODEL'] = old_model


if __name__ == '__main__': unittest.main()
