import unittest
from unittest.mock import Mock,patch
import numpy as np
from scene2.control import Robot,Stop
from scene1.motion import JOINTS

class StreamedPathTests(unittest.TestCase):
    def robot(self):
        r=Robot.__new__(Robot);r.command=np.zeros(16);state=np.zeros(16)
        r.low=np.full(16,-2.);r.high=np.full(16,2.)
        r.measured=lambda:state.copy();r.kin=Mock();r.kin.matrix.return_value=np.eye(4)
        r.check_clearance=Mock();r.log=Mock();r.joints=Mock()
        sent=[]
        def emit(goal,**kw):
            self.assertTrue(kw['backtrack'])
            self.assertLessEqual(np.max(np.abs(goal[JOINTS]-state[JOINTS])),.060001)
            self.assertLessEqual(np.max(np.abs(goal-r.command)),.012001)
            sent.append(goal.copy());r.command=goal.copy()
            state[:]+=np.clip(goal-state,-.006,.006)
        r.emit=emit
        return r,sent

    def test_long_path_streams_without_intermediate_settling(self):
        r,sent=self.robot();target=np.zeros(14);target[0]=.6;target[1]=-.3
        r.joint_path(target,'test')
        self.assertGreater(len(sent),50)
        self.assertLess(len(sent),200)
        self.assertEqual(r.joints.call_count,1)
        for q in sent:self.assertAlmostEqual(q[1],-.5*q[0])
        np.testing.assert_allclose(r.joints.call_args.args[0],target)
        self.assertEqual(r.joints.call_args.kwargs['pose_tolerance'],.006)

    def test_blocked_nominal_path_sends_nothing(self):
        for settle in (True,False):
            r,sent=self.robot();r.check_clearance.side_effect=Stop('blocked')
            with self.assertRaisesRegex(Stop,'blocked'):r.joint_path(np.ones(14)*.3,'test',settle=settle)
            self.assertEqual(sent,[]);r.joints.assert_not_called()

    def test_transit_hands_off_without_fixed_configuration_settling(self):
        r,sent=self.robot();target=np.zeros(14);target[0]=.6;target[1]=-.3
        r.joint_path(target,'transit',settle=False)
        self.assertGreater(len(sent),50)
        r.joints.assert_not_called()
        self.assertLessEqual(np.max(abs(r.measured()[JOINTS]-target)),.060001)
        self.assertEqual(r.log.call_args.args[0],'joint_path_streamed')

    def test_stationary_arm_keeps_hard_clearance_without_extra_motion_reserve(self):
        r,sent=self.robot()
        def clearance(mat,side,extra_margin=0):
            if side==1 and extra_margin>0:raise Stop('Stationary held envelope lacks extra motion reserve')
        r.check_clearance.side_effect=clearance
        target=np.zeros(14);target[0]=.3;r.joint_path(target,'observer')
        self.assertTrue(sent)
        stationary=[call for call in r.check_clearance.call_args_list if call.args[1]==1]
        self.assertTrue(stationary)
        self.assertTrue(all(call.kwargs['extra_margin']==0 for call in stationary))

    def test_preserves_initial_tracking_compensation(self):
        r,sent=self.robot();bias=np.zeros(16);bias[0]=.04;r.command=bias.copy()
        state=np.zeros(16);r.measured=lambda:state.copy()
        def emit(goal,**kw):
            self.assertLessEqual(np.max(np.abs(goal-r.command)),.012001)
            self.assertLessEqual(np.max(np.abs(goal-state)),.100001)
            sent.append(goal.copy());r.command=goal.copy()
            state[:]+=np.clip(goal-bias-state,-.006,.006)
        r.emit=emit;target=np.zeros(14);target[0]=.3
        r.joint_path(target,'test')
        self.assertGreater(sent[0][0],bias[0])
        self.assertLessEqual(abs(state[0]-.3),.061)
        r.joints.assert_called_once()

    def test_tracks_bias_that_changes_during_transport(self):
        r,sent=self.robot();state=np.zeros(16);r.measured=lambda:state.copy()
        r.command[0]=.04
        def emit(goal,**kw):
            self.assertLessEqual(np.max(np.abs(goal-r.command)),.012001)
            self.assertLessEqual(np.max(np.abs(goal-state)),.220001)
            sent.append(goal.copy());r.command=goal.copy()
            bias=np.zeros(16);bias[0]=.04+.15*state[0]
            state[:]+=np.clip(goal-bias-state,-.006,.006)
            if len(sent)>400:self.fail('Path stalled under changing tracking bias')
        r.emit=emit;target=np.zeros(14);target[0]=.6
        r.joint_path(target,'test')
        self.assertLessEqual(abs(state[0]-.6),.061)
        r.joints.assert_called_once()

