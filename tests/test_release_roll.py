import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene2.task import Sorter
from scene2.control import Stop


class ReleaseRollTests(unittest.TestCase):
    def setUp(self):
        self.sorter=Sorter.__new__(Sorter)
        self.sorter.robot=Mock(side=0)
        self.sorter.robot.solve_pose.return_value=np.zeros(7)
        self.sorter.robot.kin.matrix.return_value=np.eye(4)
        self.sorter.log=Mock()
        self.pose=np.eye(4);self.pose[:3,3]=[.5,.4,.3]
        self.payload=(np.array([.06,.01,-.02]),np.eye(3),np.array([.1,.012,.012]))

    def test_candidate_roll_preserves_payload_center_and_axis(self):
        _,path=self.sorter.plan_release_roll(self.pose,np.zeros(7),self.payload,record=False)
        center=self.pose[:3,3]+self.payload[0]
        self.assertEqual(len(path),6)
        for point,rotation in path:
            np.testing.assert_allclose(point+rotation@self.payload[0],center,atol=1e-12)
            np.testing.assert_allclose(rotation[:,0],[1,0,0],atol=1e-12)
        self.sorter.robot.pose.assert_not_called()
        self.sorter.robot.measured.assert_not_called()
        for call in self.sorter.robot.check_clearance.call_args_list:
            self.assertTrue(call.kwargs['gripper_open'])
            self.assertIs(call.kwargs['payload'],self.payload)

    def test_candidate_without_reachable_roll_is_rejected_before_release(self):
        self.sorter.robot.solve_pose.side_effect=Stop('No bounded IK for requested pose')
        with self.assertRaisesRegex(Stop,'lower finger support'):
            self.sorter.plan_release_roll(self.pose,np.zeros(7),self.payload,record=False)
        self.sorter.robot.gripper.assert_not_called()
