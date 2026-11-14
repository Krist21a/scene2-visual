import json
import unittest
from unittest.mock import Mock
from dataclasses import replace
import numpy as np
from bootstrap import ROOT
from scene2.perception import Bin,Detector,masks
from scene2.bin_geometry import cavity_bounds,propagate_cavity,register_cavity
from scene2.task import Sorter


class CavityRegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.snapshot=dict(np.load(ROOT/'tests/fixtures/c-cavity-occlusion.npz'))
        cls.colour=masks(cls.snapshot['rgb'])[1]['C']
        _,cls.bins=Detector().detect(cls.snapshot)
        d=json.loads((ROOT/'tests/fixtures/c-source-release-occupied.json').read_text())
        cls.reference=Bin(**d['empty_cavity'])
        cls.reference_bins={k:Bin(**b) for k,b in d['empty_bins'].items()}

    def register(self,snapshot=None,colour=None,reference=None,bins=None):
        return register_cavity(snapshot or self.snapshot,self.colour if colour is None else colour,
            self.bins['C'],reference or self.reference,self.reference_bins,bins or self.bins)

    def test_recorded_foreground_corner_has_independent_current_colour_support(self):
        self.assertIsNone(cavity_bounds(self.snapshot,self.colour,self.bins['C']).cavity)
        result,details=self.register()
        self.assertGreater(details['occluded_directions'],0)
        self.assertLessEqual(details['occluded_directions'],18)
        self.assertLess(details['median_error_px'],1)
        self.assertLess(details['p90_error_px'],3)
        self.assertLess(details['max_error_px'],12)
        old=np.asarray(self.reference.cavity);new=np.asarray(result.cavity)
        np.testing.assert_allclose(np.linalg.norm(np.roll(new,-1,axis=0)-new,axis=1),
                                   np.linalg.norm(np.roll(old,-1,axis=0)-old,axis=1),atol=1e-10)
        propagated=propagate_cavity(self.reference,self.reference_bins,self.bins)
        self.assertGreater(np.linalg.norm(new[1]-new[0]),np.linalg.norm(np.diff(np.asarray(propagated.cavity)[:2],axis=0))+.004)

    def test_missing_foreground_depth_cannot_excuse_missing_colour(self):
        snapshot=dict(self.snapshot,depth_m=np.full_like(self.snapshot['depth_m'],3.))
        self.assertIsNone(self.register(snapshot=snapshot))

    def test_foreground_covering_every_corner_cannot_register_a_box(self):
        snapshot=dict(self.snapshot,depth_m=np.full_like(self.snapshot['depth_m'],.2))
        self.assertIsNone(self.register(snapshot=snapshot))

    def test_disagreeing_other_bins_reject_the_anchor(self):
        bins=dict(self.bins);bins['A']=replace(bins['A'],center=(np.asarray(bins['A'].center)+[.02,0]).tolist())
        self.assertIsNone(self.register(bins=bins))

    def test_oversized_reference_is_rejected_by_current_silhouette(self):
        old=np.asarray(self.reference.cavity);center=old.mean(0)
        oversized=replace(self.reference,cavity=(center+(old-center)*1.2).tolist())
        self.assertIsNone(self.register(reference=oversized))

    def test_absent_or_displaced_colour_cannot_register_a_box(self):
        self.assertIsNone(self.register(colour=np.zeros_like(self.colour)))
        self.assertIsNone(self.register(colour=np.roll(self.colour,25,axis=1)))

    def test_current_valid_size_registration_precedes_a_smaller_unconstrained_fit(self):
        snapshot=dict(np.load(ROOT/'tests/fixtures/c-cavity-size-jitter.npz'))
        d=json.loads((ROOT/'tests/fixtures/c-cavity-size-jitter.json').read_text())
        bins={k:Bin(**v) for k,v in d['bins'].items()};reference=Bin(**d['empty'])
        direct=cavity_bounds(snapshot,masks(snapshot['rgb'])[1]['C'],bins['C'])
        self.assertIsNotNone(direct.cavity)
        sorter=Sorter.__new__(Sorter);sorter.log=Mock()
        sorter.release_empty_cavity_reference=(reference,{k:Bin(**v) for k,v in d['initial_bins'].items()})
        result=sorter.tracked_release_cavity(snapshot,bins)
        width=lambda b:np.linalg.norm(np.asarray(b.cavity[1])-b.cavity[0])
        self.assertGreater(width(result),width(direct)+.003)
        self.assertAlmostEqual(width(result),width(reference),places=10)
        self.assertEqual(sorter.log.call_args.args[0],'release_cavity_registered')
