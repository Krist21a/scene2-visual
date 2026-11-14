import unittest
from pathlib import Path
import numpy as np
from scene2.perception import Detector,point_image,masks
from scene2.task import fits_bin


class SettledScrewdriverTests(unittest.TestCase):
    def test_complete_envelopes_in_rotated_cavity(self):
        self.assertEqual(self.count('final-six-side-view.npz'),2)

    def load(self,name):return dict(np.load(Path(__file__).parent/'fixtures'/name))

    def count(self,name):
        parts,bins=Detector().detect(self.load(name),True)
        return sum(p.kind=='C' and fits_bin(p,bins['C']) for p in parts)

    def test_leaning_full_screwdriver_in_cavity(self):
        self.assertEqual(self.count('c-leaning-bin.npz'),1)

    def test_two_overlapping_handles_require_two_supported_shafts(self):
        self.assertEqual(self.count('c-overlapping-pair.npz'),2)

    def test_overlapping_handle_assignment_uses_shaft_ends(self):
        parts,bins=Detector().detect(self.load('c-overlap-near-wall.npz'),True)
        self.assertEqual(sum(p.kind=='C' for p in parts),2)
        # Separating the identities must not turn a marginal wall placement
        # into a success: one complete envelope still lacks the required margin.
        self.assertEqual(sum(p.kind=='C' and fits_bin(p,bins['C']) for p in parts),1)

    def test_one_visible_shaft_cannot_confirm_two_overlapping_handles(self):
        snapshot=self.load('c-overlapping-pair.npz')
        points,valid=point_image(snapshot);colours,_=masks(snapshot['rgb'])
        # Hide one independently measured shaft in this sensor fixture while
        # leaving both red handle silhouettes intact.
        origin=np.array([.51950509,.39264303,-.05391521])
        axis=np.array([-.80060019,-.58570196,-.12646163])
        hidden=valid&colours['A']&(np.linalg.norm(np.cross(points-origin,axis),axis=-1)<.006)
        snapshot['depth_m']=snapshot['depth_m'].copy();snapshot['depth_m'][hidden]=0
        from scene2.settled_pair import screwdriver_pair
        bins=Detector().detect(snapshot)[1]
        self.assertEqual(screwdriver_pair(snapshot,bins['C']),[])

    def test_external_wall_rest_is_not_complete(self):
        self.assertEqual(self.count('c-outside-wall.npz'),0)
        self.assertEqual(self.count('c-outside-wall-flat.npz'),0)
        self.assertEqual(self.count('c-rear-wall.npz'),0)

    def test_shaft_does_not_remove_half_the_floor(self):
        _,bins=Detector().detect(self.load('c-leaning-bin.npz'))
        self.assertLess(bins['C'].lower[1],.37)
        self.assertGreater(bins['C'].upper[1],.52)
