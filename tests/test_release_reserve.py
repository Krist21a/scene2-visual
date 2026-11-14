import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene2.control import Robot


class ReleaseReserveTests(unittest.TestCase):
    def test_bounded_ik_can_leave_a_wrist_limit_without_changing_tcp(self):
        robot=Robot.__new__(Robot)
        robot.kin=Mock(limits=[np.tile([-2.,2.],(7,1))]*2)
        def fk(q,side):
            matrix=np.eye(4);matrix[:3,3]=q[:3]
            jac=np.zeros((6,7));jac[:3,:3]=np.eye(3)
            return matrix,jac
        robot.kin.matrix_and_jacobian.side_effect=fk
        seed=np.zeros(7);seed[4:]=[1.99,-1.99,1.99]
        margin=np.r_[np.full(4,.003),np.full(3,.08)]
        q=robot.solve_pose(0,np.array([.1,0,0]),np.eye(3),seed,joint_margin=margin)
        self.assertLess(np.linalg.norm(q[:3]-[.1,0,0]),.005)
        self.assertTrue(np.all(np.abs(q[4:])<=1.92))
        ordinary=robot.solve_pose(0,np.array([.1,0,0]),np.eye(3),seed)
        self.assertTrue(np.all(np.abs(ordinary[4:])>1.95))
