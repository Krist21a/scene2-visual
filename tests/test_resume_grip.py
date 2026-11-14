import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene2.control import Robot,Stop


class ResumeGripTests(unittest.TestCase):
    def robot(self):
        r=Robot.__new__(Robot)
        r.command=np.zeros(16);r.command[7]=.73
        r.measured=Mock(return_value=r.command.copy())
        r.emit=Mock();r.log=Mock()
        return r

    def test_restore_recorded_closing_reference_not_contact_position(self):
        r=self.robot();r.restore_loaded_gripper(0,1.)
        expected=r.command.copy();expected[7]=1.
        for call in r.emit.call_args_list:np.testing.assert_array_equal(call.args[0],expected)
        self.assertEqual(r.emit.call_count,4)

    def test_checkpoint_cannot_reopen_or_exceed_gripper_limit(self):
        for command in (.70,1.1,float('nan')):
            r=self.robot()
            with self.assertRaises(Stop):r.restore_loaded_gripper(0,command)
            r.emit.assert_not_called()
