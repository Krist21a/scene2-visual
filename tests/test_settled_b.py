import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import unittest
from dataclasses import replace
import numpy as np
from scene2.perception import Detector
from scene2.task import fits_bin


class SettledBTests(unittest.TestCase):
    def test_five_recorded_frames_use_complete_contour_without_empty_box_corners(self):
        for i in range(5):
            with self.subTest(frame=i):
                snapshot=dict(np.load(Path(__file__).parent/f'fixtures/b-corner-footprint-{i}.npz'))
                parts,bins=Detector().detect(snapshot,True)
                inside=[p for p in parts if p.kind=='B' and fits_bin(p,bins['B'])]
                self.assertEqual(len(inside),2)
                for p in inside:
                    xy=np.array(p.footprint)
                    # Every observed extreme retains the existing 3mm margin,
                    # including points discarded by percentile box fitting.
                    self.assertGreaterEqual(np.min(xy-bins['B'].lower),.003)
                    self.assertGreaterEqual(np.min(np.array(bins['B'].upper)-xy),.003)

    def test_even_one_b_contour_vertex_crossing_margin_is_rejected(self):
        snapshot=dict(np.load(Path(__file__).parent/'fixtures/b-corner-footprint-4.npz'))
        parts,bins=Detector().detect(snapshot,True);b=bins['B']
        part=next(p for p in parts if p.kind=='B')
        self.assertTrue(fits_bin(part,b))
        for axis in (0,1):
            for boundary in (b.lower[axis]+.0029,b.upper[axis]-.0029):
                footprint=np.array(part.footprint);footprint[0,axis]=boundary
                self.assertFalse(fits_bin(replace(part,footprint=footprint.tolist()),b))

    def test_tilted_side_face_is_supported_by_vertical_extent(self):
        snapshot=dict(np.load(Path(__file__).parent/'fixtures/b-tilted-side.npz'))
        parts,bins=Detector().detect(snapshot,True)
        self.assertEqual(sum(p.kind=='B' and fits_bin(p,bins['B']) for p in parts),2)

    def test_narrow_visible_side_of_settled_b(self):
        snapshot=dict(np.load(Path(__file__).parent/'fixtures/final-six-side-view.npz'))
        parts,bins=Detector().detect(snapshot,True)
        self.assertEqual(sum(p.kind=='B' and fits_bin(p,bins['B']) for p in parts),2)
        self.assertFalse(any(p.kind=='B' for p in Detector().detect(snapshot)[0]))

    def test_upright_and_flat_b_both_count_inside_bin(self):
        snapshot=dict(np.load(Path(__file__).parent/'fixtures/b-upright-bin.npz'))
        parts,bins=Detector().detect(snapshot,True)
        inside=[p for p in parts if p.kind=='B' and fits_bin(p,bins['B'])]
        self.assertEqual(len(inside),2)
        source,_=Detector().detect(snapshot)
        self.assertFalse(any(p.kind=='B' for p in source))


if __name__=='__main__':unittest.main()
