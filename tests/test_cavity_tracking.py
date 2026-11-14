import unittest
from unittest.mock import Mock,patch
import json
from bootstrap import ROOT
from dataclasses import replace
import numpy as np
from scene2.perception import Bin
from scene2.bin_geometry import propagate_cavity
from scene2.task import Sorter

class CavityTrackingTests(unittest.TestCase):
    def test_occluded_release_uses_known_empty_anchor_before_recent_fit(self):
        d=json.loads((ROOT/'tests/fixtures/c-release-empty-reference.json').read_text())
        sorter=Sorter.__new__(Sorter);sorter.log=Mock()
        original=Bin(**d['empty_cavity']);original_bins={k:Bin(**v) for k,v in d['empty_bins'].items()}
        current={k:Bin(**v) for k,v in d['bins'].items()}
        sorter.release_empty_cavity_reference=(original,original_bins)
        sorter.release_cavity_reference=(Bin(**d['occluded_cavity']),current)
        visible=replace(current['C'],cavity=None)
        with patch('scene2.bin_geometry.cavity_bounds',return_value=visible),patch('scene2.task.masks',return_value=({},dict(C=None))):
            result=sorter.tracked_release_cavity(dict(rgb=None),current)
        np.testing.assert_allclose(result.cavity,d['release_bins']['C']['cavity'])
        self.assertEqual(sorter.log.call_args.kwargs['reference_kind'],'release_empty_cavity_reference')

    def test_new_direct_fit_does_not_replace_known_empty_anchor(self):
        cavity,bins,current,_=self.fixture()
        sorter=Sorter.__new__(Sorter);sorter.log=Mock()
        anchor=(cavity,bins);sorter.release_empty_cavity_reference=anchor
        newer=replace(cavity,stamp=2.)
        with patch('scene2.bin_geometry.cavity_bounds',return_value=newer),patch('scene2.task.masks',return_value=({},dict(C=None))):
            self.assertIs(sorter.tracked_release_cavity(dict(rgb=None),current),newer)
        self.assertIs(sorter.release_empty_cavity_reference,anchor)
        self.assertIs(sorter.release_cavity_reference[0],newer)

    def test_inconsistent_empty_reference_is_not_forced(self):
        cavity,bins,current,_=self.fixture()
        sorter=Sorter.__new__(Sorter);sorter.log=Mock()
        sorter.release_empty_cavity_reference=(cavity,bins)
        current['A'].center[0]+=.02
        visible=replace(current['C'],cavity=None)
        with patch('scene2.bin_geometry.cavity_bounds',return_value=visible),patch('scene2.task.masks',return_value=({},dict(C=None))):
            self.assertIs(sorter.tracked_release_cavity(dict(rgb=None),current),visible)

    def fixture(self):
        bins={k:Bin(k,[.5,y],[.43,y-.07],[.57,y+.07],0,.15,1.) for k,y in zip('ABC',(-.4,0,.4))}
        cavity=replace(bins['C'],cavity=[[.4,.3],[.6,.3],[.6,.5],[.4,.5]])
        shift=np.array([.01,-.005])
        current={k:replace(b,center=(np.asarray(b.center)+shift).tolist(),lower=(np.asarray(b.lower)+shift).tolist(),
                           upper=(np.asarray(b.upper)+shift).tolist(),stamp=2.) for k,b in bins.items()}
        return cavity,bins,current,shift

    def test_coherent_references_translate_and_inset_cavity(self):
        c,b,n,shift=self.fixture();result=propagate_cavity(c,b,n)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(np.mean(result.cavity,0),np.mean(c.cavity,0)+shift)
        self.assertLess(np.ptp(np.asarray(result.cavity)[:,0]),.2)
        self.assertEqual(result.stamp,2.)

    def test_disagreeing_reference_bins_reject_tracking(self):
        c,b,n,_=self.fixture();n['A'].center[0]+=.02
        self.assertIsNone(propagate_cavity(c,b,n))

    def test_new_floor_outside_carried_cavity_is_rejected(self):
        c,b,n,_=self.fixture();n['C'].upper[0]+=.05
        self.assertIsNone(propagate_cavity(c,b,n))

    def test_one_reference_is_insufficient(self):
        c,b,n,_=self.fixture();del n['B']
        self.assertIsNone(propagate_cavity(c,b,n))
