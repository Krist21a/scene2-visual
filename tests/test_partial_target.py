import sys,json,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np
from types import SimpleNamespace
from dataclasses import replace
from scene2.task import Sorter
from scene2.perception import Part

class PartialTargetTests(unittest.TestCase):
    def setUp(self):
        folder=Path(__file__).parent/'fixtures'
        self.context=json.loads((folder/'a-occluded-context.json').read_text())
        self.part=Part(**self.context['part'])
        self.sorter=Sorter.__new__(Sorter)
        self.sorter.views=[dict(np.load(folder/f'a-occluded-{view}.npz')) for view in ('head','wrist')]
        self.sorter.last_stamp=float(self.sorter.views[0]['stamp'])
        self.sorter.robot=SimpleNamespace(table=float(self.sorter.views[0]['table_z_base_m']),parts=[])
        self.sorter.log=lambda *a,**k:None
        self.source=np.array(self.part.center);self.source[:2]+=self.context['shift']

    def test_visible_fragment_confirms_previous_full_geometry(self):
        result=self.sorter.partial_target(self.part,self.source,self.context['table'])
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result.full_extent,self.part.full_extent)

    def test_no_visible_white_points_is_not_a_confirmation(self):
        for view in self.sorter.views:view['rgb'][:]=0
        self.assertIsNone(self.sorter.partial_target(self.part,self.source,self.context['table']))

    def test_inconsistent_prior_location_is_rejected(self):
        self.source[0]+=.05
        self.assertIsNone(self.sorter.partial_target(self.part,self.source,self.context['table']))


class OccludedCFragmentTests(unittest.TestCase):
    def setUp(self):
        folder=Path(__file__).parent/'fixtures'
        context=json.loads((folder/'c-occluded-context.json').read_text())
        self.part=Part(**context['part'])
        self.sorter=Sorter.__new__(Sorter)
        self.sorter.views=[dict(np.load(folder/f'c-occluded-{view}.npz')) for view in ('head','wrist')]
        self.sorter.last_stamp=float(self.sorter.views[0]['stamp'])
        self.table=float(self.sorter.views[0]['table_z_base_m'])
        self.parts=[Part(**p) for p in context['parts']]
        self.fragment=next(p for p in self.parts if p.kind=='C')
        self.sorter.robot=SimpleNamespace(table=self.table,parts=list(self.parts))
        self.sorter.log=lambda *a,**k:None
        self.source=np.array(self.part.center)
        self.source[:2]+=context['shift']
        # The selected part's centre is the original table plus half height.
        self.old_table=2*self.part.center[2]-self.part.top

    def test_recorded_fragment_is_replaced_by_visually_confirmed_full_target(self):
        self.assertGreater(np.linalg.norm(np.array(self.fragment.center)-self.source),.030)
        result=self.sorter.partial_target(self.part,self.source,self.old_table)
        self.assertIsNotNone(result)
        self.assertFalse(any(p is self.fragment for p in self.sorter.robot.parts))
        self.assertTrue(all(any(p is q for p in self.sorter.robot.parts)
                            for q in self.parts if q.kind!='C'))
        np.testing.assert_allclose(result.full_extent,self.part.full_extent)

    def test_nearby_independent_handle_support_remains_an_obstacle(self):
        other=replace(self.fragment,occupied_xy=(np.array(self.fragment.occupied_xy)+[0,.012]).tolist())
        self.sorter.robot.parts.append(other)
        self.sorter.partial_target(self.part,self.source,self.old_table)
        self.assertTrue(any(p is other for p in self.sorter.robot.parts))

    def test_sparse_or_incomplete_overlap_is_not_enough_to_remove_obstacle(self):
        sparse=replace(self.fragment,occupied_xy=self.fragment.occupied_xy[:20])
        support=np.array(self.fragment.occupied_xy)
        support[:15]+=[0,.04]
        mixed=replace(self.fragment,occupied_xy=support.tolist())
        self.sorter.robot.parts.extend([sparse,mixed])
        self.sorter.partial_target(self.part,self.source,self.old_table)
        self.assertTrue(all(any(p is q for p in self.sorter.robot.parts) for q in (sparse,mixed)))

    def test_no_fresh_visual_confirmation_keeps_fragment(self):
        for view in self.sorter.views:view['rgb'][:]=0
        self.assertIsNone(self.sorter.partial_target(self.part,self.source,self.old_table))
        self.assertTrue(any(p is self.fragment for p in self.sorter.robot.parts))

if __name__=='__main__':unittest.main()
