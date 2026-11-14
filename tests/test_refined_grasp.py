import unittest
from unittest.mock import Mock
from bootstrap import ROOT
import numpy as np
from scene2.task import Sorter
from scene2.control import Stop
from scene2.perception import Part,Bin


class RefinedGraspTests(unittest.TestCase):
    def test_new_rgbd_restarts_safe_height_search_instead_of_inheriting_raised_grasp(self):
        s=Sorter.__new__(Sorter);s.log=Mock()
        part=Part('A',[.3,-.2,.01],[1,0],[.08,.04],.02,[],1.,1.)
        bins={k:Bin(k,[.5,y],[.4,y-.1],[.6,y+.1],.01,.16,1.) for k,y in zip('ABC',[-.4,0.,.4])}
        s.robot=Mock(table=0.,bins=bins,parts=[part]);pose=np.eye(4);pose[:3,3]=[.3,-.2,.12]
        s.robot.pose.return_value=pose
        s.observe=Mock(return_value=({'table_z_base_m':0.},[part],bins))
        # The initial obscured observation required a raised coarse grasp.
        # Stop at refinement so the test never pretends to grasp or to place.
        s.robot.observation_hover.return_value=(np.eye(3),np.array([.3,-.2,.025]))
        s.robot.orient_for_grasp.side_effect=Stop('capture refined search')
        with self.assertRaisesRegex(Stop,'capture refined search'):
            s.pick_place(part,bins['A'],{'table_z_base_m':0.})
        refined=s.robot.orient_for_grasp.call_args
        self.assertEqual(refined.kwargs['height_adjust'],0.)
        self.assertTrue(callable(refined.kwargs['validator']))
        s.robot.gripper.assert_not_called()
