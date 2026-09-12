import json
import unittest
from replay import replay


class ReplayTests(unittest.TestCase):
    def test_deterministic_replay_and_deadline(self):
        lines=[]
        for i in range(90):
            now=i/60
            obs=[{'camera':'room','joint':'hip','position':[.2,1,0],'confidence':.95,'timestamp':now}] if i<60 else []
            lines.append(json.dumps({'time':now,'baseline':{'hip':[0,1,0]},'observations':obs,'enabled':True}))
        first=list(replay(lines))
        self.assertEqual(first,list(replay(lines)))
        self.assertGreater(first[59]['joints']['hip']['fused'][0], .15)
        self.assertEqual(first[-1]['joints']['hip']['fused'],[0,1,0])

    def test_malformed_frame_reports_line(self):
        with self.assertRaisesRegex(ValueError,'line 1'):
            list(replay(['{"enabled":"true"}']))
