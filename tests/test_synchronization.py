import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene2.synchronization import image_arm_pose


def feedback(stamp, value):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:stamp)),
                           joint_data=SimpleNamespace(joint_q=np.arange(30)+value))


class ImageArmSynchronizationTests(unittest.TestCase):
    def setUp(self):
        self.scene=SimpleNamespace(lock=threading.Lock(),
            joint_history=[feedback(10.,0),feedback(10.02,100),feedback(10.3,200)])
        self.kin=Mock()

    def test_image_uses_nearest_feedback_instead_of_latest(self):
        for side in (0,1):
            _,dt=image_arm_pose(self.scene,self.kin,10.021,side)
            q,selected_side=self.kin.matrix.call_args.args
            np.testing.assert_equal(q,np.arange(13+7*side,20+7*side)+100)
            self.assertEqual(selected_side,side)
            self.assertAlmostEqual(dt,.001)

    def test_stale_or_missing_feedback_rejected(self):
        with self.assertRaisesRegex(ValueError,'timestamps'):image_arm_pose(self.scene,self.kin,11.,0)
        self.scene.joint_history=[]
        with self.assertRaises(ValueError):image_arm_pose(self.scene,self.kin,10.,0)
        self.kin.matrix.assert_not_called()

    def test_invalid_joint_vector_rejected(self):
        self.scene.joint_history[1].joint_data.joint_q=np.full(30,np.nan)
        with self.assertRaisesRegex(ValueError,'feedback'):image_arm_pose(self.scene,self.kin,10.02,0)
