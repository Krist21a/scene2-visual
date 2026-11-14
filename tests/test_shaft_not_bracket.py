import unittest
from pathlib import Path
import numpy as np
from scene2.perception import Detector


class ShaftNotBracketTests(unittest.TestCase):
    def test_real_head_frame_keeps_brackets_rejects_depth_broadened_shaft(self):
        snapshot=dict(np.load(Path(__file__).parent/'fixtures/c-shaft-false-a.npz'))
        parts,_=Detector().detect(snapshot)
        brackets=[p for p in parts if p.kind=='A']
        self.assertEqual(len(brackets),2)
        self.assertTrue(all(p.center[1]<0 for p in brackets))
