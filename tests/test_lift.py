import unittest
from unittest.mock import Mock
from types import SimpleNamespace
from bootstrap import ROOT
import numpy as np
from scipy.spatial.transform import Rotation
from scene2.control import Stop
from scene2.task import Sorter


class LiftTests(unittest.TestCase):
    def sorter(self):
        s=Sorter.__new__(Sorter);s.log=Mock()
        r=Mock(table=0.,payload=(np.zeros(3),np.eye(3),np.array([.04,.02,.01])))
        s.robot=r
        pose=np.eye(4);pose[:3,3]=[.31,-.22,.017]
        pose[:3,:3]=Rotation.from_euler('x',.4).as_matrix()
        grasp=pose.copy();r.pose.side_effect=lambda side:pose.copy()
        def move(side,point,rotation,stage,**kwargs):
            pose[:3,3]=point;pose[:3,:3]=rotation
        r.cartesian.side_effect=move
        return s,grasp

    def test_a_lifts_measured_pose_vertically_before_alignment(self):
        s,grasp=self.sorter();source=np.array([.3,-.2,.015]);desired=np.eye(3)
        s.lift_grasp(SimpleNamespace(kind='A'),1,source,desired,grasp)
        calls=s.robot.cartesian.call_args_list
        self.assertEqual(calls[-1].args[3],'lift_align')
        heights=[grasp[2,3]]
        for call in calls[:-1]:
            np.testing.assert_allclose(call.args[1][:2],grasp[:2,3])
            np.testing.assert_allclose(call.args[2],grasp[:3,:3])
            heights.append(call.args[1][2])
        self.assertTrue(np.all(np.diff(heights)>0))
        self.assertLessEqual(max(np.diff(heights)),.020001)
        self.assertEqual(calls[-2].kwargs['settle_frames'],5)
        self.assertEqual(s.robot.check_clearance.call_count,21)
        self.assertTrue(all(c.args[0][2,3]>=.10-1e-12 for c in s.robot.check_clearance.call_args_list))

    def test_failed_alignment_keeps_loaded_hand_without_rotation_or_opening(self):
        s,grasp=self.sorter();s.robot.check_clearance.side_effect=Stop('table reserve')
        with self.assertRaisesRegex(Stop,'table reserve'):
            s.lift_grasp(SimpleNamespace(kind='A'),1,np.array([.3,-.2,.015]),np.eye(3),grasp)
        self.assertTrue(all(c.args[3]=='lift' for c in s.robot.cartesian.call_args_list))
        s.robot.gripper.assert_not_called()
