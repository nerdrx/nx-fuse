import time
import threading
import unittest
from unittest.mock import patch
from shadow_session import ShadowSession
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipIf(np is None, 'OpenCV extras unavailable')
class ShadowSessionTests(unittest.TestCase):
    def setUp(self):
        from test_shadow_pose import ShadowPoseTests
        from types import SimpleNamespace
        fixture=ShadowPoseTests();fixture.setUp()
        self.fixture=fixture
        self.profile={**fixture.alignment,'validated':True,'camera_id':'camera','camera_epoch':'one',
                      'image_size':[640,480],'generation':'1','lens_signature':'fixture'}
        self.lens={**fixture.lens,'validated':True,'camera_id':'camera'}
        self.estimation=True
        self.sequence=0
        self.generation='1'
        def frame(camera):
            self.sequence+=1
            return dict(estimate=fixture.estimate,time_ns=time.monotonic_ns()-25_000_000,
                        sequence=self.sequence,epoch='one',image_size=[640,480])
        self.camera=SimpleNamespace(status=lambda _:dict(streaming=True,estimation=self.estimation,
                                    epoch='one',image_size=[640,480]),estimate_snapshot=frame)
        self.tracking=SimpleNamespace(history_snapshot=lambda _:
            dict(generation=self.generation,match_error_ms=5,records=[dict(joint='head',**fixture.head),
                 dict(joint='hip',position=[.02,.35,0])]))
        self.session=ShadowSession(self.camera,SimpleNamespace(snapshot=lambda:dict(profile=self.lens)),
                                   SimpleNamespace(snapshot=lambda:dict(profile=self.profile)),self.tracking)
    def tearDown(self): self.session.close()
    def wait_for(self, predicate):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            state=self.session.snapshot()
            if predicate(state): return state
            time.sleep(.02)
        self.fail(self.session.snapshot())
    def test_observations_recording_and_immediate_expiry(self):
        self.assertFalse(self.session.snapshot()['enabled'])
        self.session.set_enabled('camera',True)
        state=self.wait_for(lambda s:s['available'])
        self.assertEqual(state['mode'],'shadow')
        hip=next(j for j in state['joints'] if j['joint']=='hip')
        self.assertAlmostEqual(hip['residual_m'],.03,places=5)
        self.assertFalse(any(j['joint'] in ('head','left_hand','right_hand') for j in state['joints']))
        self.assertEqual(self.session.recording()['frames'],[])
        self.session.recording_command('start')
        self.wait_for(lambda s:s['recorded_frames']>0)
        self.session.recording_command('stop')
        log=self.session.recording()
        self.assertNotIn('jpeg',str(log))
        self.assertEqual(log['frames'][0]['generation'],'1')
        self.estimation=False
        self.assertFalse(self.session.snapshot()['available'])
        self.session.set_enabled('camera',False)
        self.assertEqual(self.session.snapshot()['joints'],[])
        self.session.recording_command('clear')
        self.assertEqual(self.session.recording()['frames'],[])
    def test_generation_mismatch_and_stale_observation(self):
        self.generation='2'
        self.session.set_enabled('camera',True)
        self.wait_for(lambda s:'VR space changed' in s['reason'])
        self.assertFalse(self.session.snapshot()['available'])
        self.generation='1'
        self.wait_for(lambda s:s['available'])
        with self.session._lock: self.session._result['_time_ns']-=1_000_000_000
        self.assertFalse(self.session.snapshot()['available'])
    def test_disable_cannot_publish_inflight_and_recording_cap(self):
        import shadow_pose
        entered,release=threading.Event(),threading.Event()
        real=shadow_pose.reconstruct
        def delayed(*args):
            entered.set();release.wait(2);return real(*args)
        with patch('shadow_pose.reconstruct',delayed):
            self.session.set_enabled('camera',True)
            self.assertTrue(entered.wait(1))
            self.session.set_enabled('camera',False);release.set()
            time.sleep(.15)
            self.assertFalse(self.session.snapshot()['available'])
        self.session.set_enabled('camera',True)
        with self.session._lock: self.session._frames=[{}]*599
        self.session.recording_command('start')
        state=self.wait_for(lambda s:s['recording_full'])
        self.assertEqual(state['recorded_frames'],600)
        self.assertFalse(state['recording'])
        with self.assertRaises(ValueError):self.session.recording_command('start')


if __name__=='__main__':unittest.main()
