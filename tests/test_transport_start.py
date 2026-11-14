import json
import unittest
from unittest.mock import Mock, patch
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot, Stop
from scene2.perception import Part, Bin


class TransportStartTests(unittest.TestCase):
    def robot(self):
        d=json.loads((ROOT/'tests/fixtures/c-start-reference-c22.json').read_text())
        r=Robot.__new__(Robot);r.side=0;r.stage='replay';r.table=d['table'];r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.command=np.array(d['command']);state=np.array(d['measured'])
        r.measured=lambda:state.copy()
        r.low=np.r_[r.kin.limits[0][:,0],0,r.kin.limits[1][:,0],0]
        r.high=np.r_[r.kin.limits[0][:,1],1,r.kin.limits[1][:,1],1]
        r.payload=tuple(np.array(v) for v in d['payload']);r.payload_shaft_sign=1.;r.held=True
        r.target=Part(**d['part']);r.parts=[Part(**p) for p in d['parts']]
        r.bins={k:Bin(**b) for k,b in d['bins'].items()};r.settled_parts=[]
        return r,state

    def test_clear_measured_pose_does_not_hide_blocked_reference(self):
        r,state=self.robot()
        r.check_clearance(r.kin.matrix(state[:7],0),0,payload=r.payload,extra_margin=.010)
        with self.assertRaisesRegex(Stop,'wall'):r.check_transport_start(state,r.payload)

    def test_retreat_clears_first_command_and_keeps_closed_loop_limits(self):
        r,state=self.robot();bias=r.command[:7]-state[:7]
        point,rotation=r.transport_clearance_goal(state,r.payload)
        self.assertLess(point[0],r.kin.matrix(state[:7],0)[0,3])
        previous=r.command.copy();steps=[]
        def emit(goal,**kw):
            nonlocal previous
            self.assertLessEqual(np.max(abs(goal[:7]-previous[:7])),.01200001)
            self.assertLessEqual(np.max(abs(goal[:7]-state[:7])),.25)
            r.check_clearance(r.kin.matrix(state[:7],0),0,payload=r.payload)
            for alpha in (.5,1.):
                r.check_clearance(r.kin.matrix(state[:7]+(goal[:7]-state[:7])*alpha,0),0,payload=r.payload)
            r.command=goal.copy();previous=goal.copy()
            # Recorded static MPC bias; this validates feedback logic, not physics.
            state[:7]=goal[:7]-bias;steps.append(goal.copy())
        r.emit=emit
        r.cartesian(0,point,rotation,'raise_transport_clearance',position_tolerance=.003)
        self.assertGreater(len(steps),1)
        r.check_transport_start(state,r.payload)

    def test_no_clear_first_step_never_moves_or_opens(self):
        r,state=self.robot();r.emit=Mock();r.gripper=Mock()
        with patch.object(r,'check_clearance',side_effect=Stop('wall')):
            with self.assertRaisesRegex(Stop,'No bounded'):r.transport_clearance_goal(state,r.payload)
        r.emit.assert_not_called();r.gripper.assert_not_called()


if __name__=='__main__':unittest.main()
