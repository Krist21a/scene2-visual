import unittest
from unittest.mock import Mock
from bootstrap import ROOT
from scene2.handover import Handover
from scene2.control import Robot,Stop


class HandoverTests(unittest.TestCase):
    def robot(self):
        r=Robot.__new__(Robot);r.emit=Mock();r.handover=Handover(0,1,Mock(),clock=lambda:0.)
        return r

    def test_contact_without_visual_confirmation_never_opens_donor(self):
        r=self.robot();r.handover.receiver_closed()
        with self.assertRaises(Stop):r.handover.observe(1.,.5,1.,0,object())
        with self.assertRaises(Stop):r.gripper(0,False,'handover_release')
        r.emit.assert_not_called()
        self.assertEqual(r.handover.state,'failed')

    def test_stale_or_incomplete_confirmation_keeps_donor(self):
        for frames in (0,1,2):
            r=self.robot();r.handover.receiver_closed()
            for i in range(frames):r.handover.observe(float(i),.5,1.,20,object())
            with self.assertRaises(Stop):r.gripper(0,False,'handover_release')
            r.emit.assert_not_called()
        r=self.robot();r.handover.receiver_closed()
        for i in range(3):r.handover.observe(float(i),.5,1.,20,object())
        r.handover.clock=lambda:2.
        with self.assertRaises(Stop):r.gripper(0,False,'handover_release')
        r.emit.assert_not_called()

    def test_three_new_supported_frames_authorize_only_donor_release(self):
        r=self.robot();r.handover.receiver_closed();payload=object()
        for i in range(3):r.handover.observe(float(i),.5,1.,20,payload)
        r.handover.authorize_open(0)
        self.assertIs(r.handover.payload,payload)
        self.assertEqual(r.handover.state,'confirmed')

    def test_duplicate_frame_cannot_complete_transfer(self):
        r=self.robot();r.handover.receiver_closed()
        r.handover.observe(1.,.5,1.,20,object())
        with self.assertRaisesRegex(Stop,'Repeated'):r.handover.observe(1.,.5,1.,20,object())
        with self.assertRaises(Stop):r.gripper(0,False,'handover_release')
        r.emit.assert_not_called()

    def test_authorization_precedes_release_and_completion(self):
        r=self.robot();r.handover.receiver_closed()
        for i in range(3):r.handover.observe(float(i),.5,1.,20,object())
        r.handover.begin_release()
        r.handover.clock=lambda:4.
        r.handover.transferred()
        self.assertEqual(r.handover.state,'transferred')
