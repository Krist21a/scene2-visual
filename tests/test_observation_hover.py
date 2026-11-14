import unittest
from unittest.mock import Mock
from types import SimpleNamespace
import numpy as np
from scene2.control import Robot,Stop


class ObservationHoverTests(unittest.TestCase):
    def robot(self):
        r=Robot.__new__(Robot);r.measured=lambda:np.zeros(16);r.log=Mock()
        r.kin=Mock();r.kin.matrix.return_value=np.eye(4)
        r.solve_pose=Mock(return_value=np.ones(7)*.1)
        r.check_clearance=Mock();r.joint_path=Mock()
        return r

    def test_observation_only_plans_above_table_and_checks_entire_approach(self):
        r=self.robot();part=SimpleNamespace(kind='A',axis=[1.,0.],center=[.3,-.2,.01])
        hover=np.array([.3,-.2,.12])
        r.observation_hover(1,part,hover)
        self.assertTrue(all(np.array_equal(c.args[1],hover) for c in r.solve_pose.call_args_list))
        self.assertGreaterEqual(r.check_clearance.call_count,31)
        self.assertTrue(all(c.kwargs['extra_margin']==.005 for c in r.check_clearance.call_args_list))
        r.joint_path.assert_called_once()

    def test_blocked_observation_path_emits_no_motion(self):
        r=self.robot();r.check_clearance.side_effect=Stop('obstacle')
        with self.assertRaisesRegex(Stop,'No checked observation hover'):
            r.observation_hover(1,SimpleNamespace(kind='A',axis=[1.,0.],center=[.3,-.2,.01]),np.array([.3,-.2,.12]))
        r.joint_path.assert_not_called()


if __name__=='__main__':unittest.main()
