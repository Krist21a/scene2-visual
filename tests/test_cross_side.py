import unittest
from unittest.mock import Mock
from bootstrap import ROOT
from scene2.control import Stop
from scene2.perception import Bin,Part
from scene2.task import Sorter


class CrossSideTests(unittest.TestCase):
    def test_cross_side_target_checks_direct_routes_before_any_motion(self):
        s=Sorter.__new__(Sorter);s.log=Mock();s.plan_place=Mock()
        target=Bin('A',[.5,-.4],[.4,-.5],[.6,-.3],0,.15,1.)
        s.robot=Mock(table=0.,bins={'A':target})
        s.robot.orient_for_grasp.side_effect=[Stop('unreachable'),None]
        part=Part('A',[.3,.2,.02],[1.,0.],[.07,.03],.04,[],1.,1.)
        self.assertEqual(s.select_arm(part,target),1)
        self.assertEqual([c.args[0] for c in s.robot.orient_for_grasp.call_args_list],[0,1])
        self.assertTrue(all(c.kwargs['execute'] is False for c in s.robot.orient_for_grasp.call_args_list))
        s.robot.gripper.assert_not_called()
        s.robot.joints.assert_not_called()

    def test_unreachable_target_requires_checked_handover(self):
        s=Sorter.__new__(Sorter);s.log=Mock()
        target=Bin('A',[.5,-.4],[.4,-.5],[.6,-.3],0,.15,1.)
        s.robot=Mock(table=0.,bins={'A':target});s.robot.orient_for_grasp.side_effect=Stop('unreachable')
        part=Part('A',[.3,.2,.02],[1.,0.],[.07,.03],.04,[],1.,1.)
        with self.assertRaisesRegex(Stop,'rendezvous'):s.select_arm(part,target)
        s.robot.gripper.assert_not_called()
