import math
import unittest
from fusion import Fusion, Observation


class FusionTests(unittest.TestCase):
    def test_assistance_failover_and_protected_head(self):
        f = Fusion()
        base = {'hip':(0,1,0), 'head':(0,2,0)}
        for i in range(121):
            now = i/60
            out = f.step(base,[Observation('a',j,(.2,p[1],0),.95,now) for j,p in base.items()],now,True)
        self.assertGreater(out['hip']['fused'][0], .16)
        self.assertEqual(out['head']['fused'], [0,2,0])
        last = out['hip']['fused'][0]
        out = f.step(base,[],2+1/60,True)
        self.assertLess(out['hip']['fused'][0],last)
        self.assertGreater(out['hip']['fused'][0],last-.02)
        for i in range(1,121):
            out = f.step(base,[],2+i/60,True)
        self.assertLess(abs(out['hip']['fused'][0]), .0001)
        self.assertEqual(f.step(base,[],4,False)['hip']['fused'], [0,1,0])

    def test_reject_stale_future_nonfinite_outlier_and_conflict(self):
        bad = [Observation('a','hip',(.1,1,0),.95,-1),
               Observation('a','hip',(.1,1,0),.95,1),
               Observation('a','hip',(math.nan,1,0),.95,0),
               Observation('a','hip',(.1,1,0),math.nan,0),
               Observation('a','hip',(2,1,0),.95,0),
               Observation('a','hip',(.1,1,0),.4,0)]
        for observations in [[o] for o in bad]+[[Observation('a','hip',(.2,1,0),.95,0),Observation('b','hip',(-.2,1,0),.95,0)]]:
            f = Fusion()
            f.step({'hip':(0,1,0)},[],0,True)
            self.assertEqual(f.step({'hip':(0,1,0)},observations,.01,True)['hip']['fused'],[0,1,0])

    def test_duplicate_camera_does_not_increase_influence(self):
        f,g = Fusion(),Fusion()
        for i in range(60):
            now=i/60
            a=Observation('a','hip',(.1,1,0),.9,now)
            b=Observation('b','hip',(.2,1,0),.9,now)
            self.assertEqual(f.step({'hip':(0,1,0)},[a,b],now,True),g.step({'hip':(0,1,0)},[a,a,a,b],now,True))

    def test_reset_and_bad_base(self):
        f=Fusion()
        with self.assertRaises(ValueError):
            f.step({'hip':(math.inf,0,0)},[],0)
        f.corrections['hip']=(.2,0,0)
        f.last_time=20
        self.assertEqual(f.step({'hip':(0,1,0)},[],1,True)['hip']['fused'],[0,1,0])
        f.reset()
        self.assertEqual(f.corrections,{})

    def test_long_process_gap_discards_old_correction(self):
        f=Fusion()
        for i in range(121):
            now=i/60
            f.step({'hip':(0,1,0)},[Observation('a','hip',(.2,1,0),.95,now)],now,True)
        self.assertEqual(f.step({'hip':(0,1,0)},[],12,True)['hip']['fused'],[0,1,0])


if __name__ == '__main__':
    unittest.main()
