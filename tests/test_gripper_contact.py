import unittest
import numpy as np
from scene2.control import Robot


class GripperContactTests(unittest.TestCase):
    def run_close(self,contact):
        class FeedbackRobot:
            command=np.zeros(16)
            actual=np.zeros(16)
            events=[]
            def measured(self):return self.actual.copy()
            def emit(self,goal):
                self.command=goal.copy()
                self.actual[7]=max(0.,min(goal[7]-.04,contact))
            def log(self,event,**values):self.events.append(event)
        r=FeedbackRobot()
        Robot.gripper(r,0,True,'close',contact_stop=True)
        return r

    def test_contact_limits_further_compression(self):
        r=self.run_close(.60)
        self.assertIn('gripper_contact_stop',r.events)
        self.assertGreater(r.command[7],.60)
        self.assertLess(r.command[7],.75)

    def test_freely_moving_jaws_reach_closed_command(self):
        r=self.run_close(1.)
        self.assertNotIn('gripper_contact_stop',r.events)
        self.assertEqual(r.command[7],1.)
