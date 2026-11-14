import unittest
import numpy as np
from bootstrap import ROOT
from scene2.perception import Detector,point_image
from scene2.task import fits_bin


class SettledShapeTests(unittest.TestCase):
    def setUp(self):
        self.s=dict(np.load(ROOT/'tests/fixtures/a-upright-bin.npz'))

    def test_upright_bracket_has_complete_shape_inside_bin(self):
        parts,bins=Detector().detect(self.s,include_bins=True)
        inside=[p for p in parts if p.kind=='A' and fits_bin(p,bins['A'])]
        self.assertEqual(len(inside),1)
        self.assertGreater(inside[0].top-bins['A'].floor,.07)
        table_parts,_=Detector().detect(self.s)
        self.assertEqual(sum(p.kind=='A' for p in table_parts),1)

    def test_dark_shape_without_white_support_is_not_a(self):
        _,bins=Detector().detect(self.s)
        points,valid=point_image(self.s);b=bins['A']
        region=valid.copy()
        for j in (0,1):region&=(points[:,:,j]>b.lower[j])&(points[:,:,j]<b.upper[j])
        region&=points[:,:,2]>b.floor+.003
        self.s['rgb'][region]=[70,70,70]
        parts,bins=Detector().detect(self.s,include_bins=True)
        self.assertFalse(any(p.kind=='A' and fits_bin(p,bins['A']) for p in parts))
