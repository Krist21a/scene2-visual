import unittest
from unittest.mock import Mock,patch
import numpy as np
from bootstrap import ROOT
from scene1.motion import MotionFilter
from scene2.control import Robot,Stop


class ReferenceGovernorTests(unittest.TestCase):
    def robot(self):
        r=Robot.__new__(Robot)
        r.run_id='epoch';r.description='urdf'
        r.rospy=Mock();r.rospy.get_param_cached.side_effect=lambda k: 'epoch' if k=='/run_id' else 'urdf'
        r.command=np.zeros(16);r.command[0]=.009
        r.filter=MotionFilter(np.full(16,-2.),np.full(16,2.),1.,.12)
        r.filter.reset(r.command)
        r.measured=Mock(return_value=np.zeros(16));r.retained=False;r.table=0.
        def fk(q,side):
            mat=np.eye(4);mat[2,3]=q[0];return mat
        r.kin=Mock();r.kin.matrix.side_effect=fk
        def clearance(mat,side):
            if mat[2,3]>.01:raise Stop('Hand/table clearance failed')
        r.check_clearance=Mock(side_effect=clearance)
        r.env=Mock();r.log=Mock();r.steps=0;r.stage='descend'
        return r

    @patch('scene2.control.time.sleep')
    def test_unsafe_reference_is_reduced_before_it_is_sent(self,_):
        r=self.robot();goal=r.command.copy();goal[0]=.021
        r.emit(goal,backtrack=True)
        sent=r.env.exec_action.call_args.args[0]
        self.assertGreater(sent[0],.009)
        self.assertLessEqual(sent[0],.01)
        self.assertLessEqual(abs(sent[0]-.009),.012)
        np.testing.assert_equal(r.filter.previous,sent)

    @patch('scene2.control.time.sleep')
    def test_unsafe_measured_position_still_stops_without_sending(self,_):
        r=self.robot();measured=np.zeros(16);measured[0]=.011;r.measured.return_value=measured
        previous=r.filter.previous.copy()
        with self.assertRaisesRegex(Stop,'table'):r.emit(np.zeros(16),backtrack=True)
        r.env.exec_action.assert_not_called()
        np.testing.assert_equal(r.filter.previous,previous)

    @patch('scene2.control.time.sleep')
    def test_rejected_reference_does_not_advance_filter(self,_):
        r=self.robot();previous=r.filter.previous.copy();goal=previous.copy();goal[0]=.021
        with self.assertRaisesRegex(Stop,'table'):r.emit(goal)
        r.env.exec_action.assert_not_called()
        np.testing.assert_equal(r.filter.previous,previous)

    @patch('scene2.control.time.sleep')
    def test_can_hold_safe_reference_while_feedback_catches_up(self,_):
        r=self.robot();r.command[0]=.01;r.filter.reset(r.command)
        goal=r.command.copy();goal[0]=.022
        r.emit(goal,backtrack=True)
        self.assertEqual(r.env.exec_action.call_args.args[0][0],.01)
