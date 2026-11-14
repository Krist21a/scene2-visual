import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from bootstrap import ROOT
import numpy as np
from scene2.perception import Detector,Part
from scene2.task import Sorter,fits_bin


class TargetSelectionTests(unittest.TestCase):
    def test_recorded_wrist_bin_object_is_excluded_from_sources_but_kept_as_obstacle(self):
        views={name:dict(np.load(ROOT/f'tests/fixtures/completed-b-wrist-{name}.npz'))
               for name in ('head','wrist0','wrist1')}
        detector=Detector();placed,bins=detector.detect(views['head'],True)
        wrist_parts=[p for side in (0,1) for p in detector.detect(views[f'wrist{side}'])[0]]
        self.assertTrue(any(p.kind=='B' and bins['B'].contains(p.center[:2]) for p in wrist_parts))
        s=Sorter.__new__(Sorter);s.robot=SimpleNamespace();s.detector=detector
        s.scene=Mock();s.scene.read.return_value=views['head']
        s.wrist=Mock();s.wrist.read.side_effect=lambda side,table:views[f'wrist{side}']
        s.last_stamp=None;s.save=Mock();s.log=Mock()
        s.bin_parts={k:[p for p in placed if p.kind==k and fits_bin(p,bins[k])] for k in 'ABC'}
        _,sources,_=s.observe('selection_replay')
        self.assertFalse(any(p.kind=='B' for p in sources))
        self.assertTrue(any(p.kind=='C' for p in sources))
        self.assertEqual(sum(p.kind=='B' for p in s.robot.settled_parts),2)

    def parts(self):
        return [Part(k,[.3,0,.02],[1,0],[.06,.03],.04,[],1.,1.) for k in 'ABC']

    def test_after_five_placements_only_last_c_is_eligible(self):
        s=Sorter.__new__(Sorter);s.initial_counts=dict.fromkeys('ABC',0)
        s.completed=[dict(kind=k) for k in 'BBAAC']
        self.assertEqual([p.kind for p in s.pending_targets(self.parts())],['C'])
        self.assertEqual(s.initial_counts,dict.fromkeys('ABC',0))

    def test_initial_occupancy_and_new_placements_both_reduce_remaining_work(self):
        s=Sorter.__new__(Sorter);s.initial_counts=dict(A=1,B=2,C=0)
        s.completed=[dict(kind='A')]
        self.assertEqual([p.kind for p in s.pending_targets(self.parts())],['C'])
        self.assertEqual(s.pending_targets(self.parts(),'B'),[])
        s.completed.extend([dict(kind='C'),dict(kind='C')])
        self.assertEqual(s.pending_targets(self.parts()),[])


if __name__=='__main__':unittest.main()
