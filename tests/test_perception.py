import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import unittest
from collections import Counter
import numpy as np
from scene2.perception import Detector, point_image


class SensorReplayTests(unittest.TestCase):
    def setUp(self):
        self.s=dict(np.load(ROOT/'tests/fixtures/scene2-head.npz'))

    def test_six_distinct_parts_three_bins(self):
        parts,bins=Detector().detect(self.s)
        self.assertEqual(Counter(p.kind for p in parts),{'A':2,'B':2,'C':2})
        self.assertEqual(set(bins),{'A','B','C'})
        for i,p in enumerate(parts):
            for q in parts[i+1:]:
                self.assertGreater(np.linalg.norm(np.array(p.center)[:2]-q.center[:2]),.05)

    def test_shaft_included_without_moving_handle_grasp(self):
        parts,_=Detector().detect(self.s)
        for p in parts:
            if p.kind=='C':
                self.assertGreater(p.full_extent[0],.18)
                self.assertLess(p.extent[0],.10)
                self.assertGreater(np.linalg.norm(np.array(p.full_center)-p.center),.035)

    def test_no_valid_depth_no_targets(self):
        self.s['depth_m']=np.zeros_like(self.s['depth_m'])
        parts,bins=Detector().detect(self.s)
        self.assertEqual(parts,[])
        self.assertEqual(bins,{})

    def test_missing_red_handle_not_invented(self):
        rgb=self.s['rgb'].copy()
        red=(rgb[:,:,0]>rgb[:,:,1]*1.8)&(rgb[:,:,0]>rgb[:,:,2]*1.8)
        rgb[red]=[20,80,20]
        self.s['rgb']=rgb
        parts,_=Detector().detect(self.s)
        self.assertFalse(any(p.kind=='C' for p in parts))

    def test_camera_translation_moves_detections(self):
        original,_=Detector().detect(self.s)
        self.s['camera_to_base']=self.s['camera_to_base'].copy()
        self.s['camera_to_base'][0,3]+=.03
        shifted,_=Detector().detect(self.s)
        self.assertEqual(len(original),len(shifted))
        for a,b in zip(original,shifted):
            np.testing.assert_allclose(np.array(b.center)-a.center,[.03,0,0],atol=1e-7)

    def test_bins_are_floor_regions_not_transparent_wall_depth(self):
        _,bins=Detector().detect(self.s)
        for b in bins.values():
            dims=np.array(b.upper)-b.lower
            self.assertTrue(np.all(dims>.15))
            self.assertTrue(np.all(dims<.25))
            self.assertAlmostEqual(b.floor,float(self.s['table_z_base_m'])+.01)
            self.assertFalse(b.contains(np.array(b.upper)+.001))
            self.assertTrue(b.contains(b.center,.02))


if __name__=='__main__': unittest.main()
