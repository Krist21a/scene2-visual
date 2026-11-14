import json
import unittest
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene1.motion import JOINTS
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin


class EmptyReturnTests(unittest.TestCase):
    def robot(self):
        d=json.loads((ROOT/'tests/fixtures/c-empty-return-c38.json').read_text())
        r=Robot.__new__(Robot);r.side=0;r.held=False;r.payload=None;r.target=None;r.stage='retreat_configuration'
        r.table=d['table'];r.command=np.asarray(d['command']);state=np.asarray(d['measured'])
        r.measured=lambda:state.copy();r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.parts=[Part(**p) for p in d['parts']];r.bins={k:Bin(**v) for k,v in d['bins'].items()}
        r.settled_parts=[Part(**p) for p in d['settled']]
        r.emit=Mock(side_effect=AssertionError('Planning sent a command'))
        return r,state,np.asarray(d['destination'])

    def test_recorded_joint_return_hits_wall_but_pose_corridor_clears(self):
        r,state,target=self.robot();start=state[:7].copy();bias=r.command[:7]-start
        self.assertEqual(len(r.settled_parts),1)
        with self.assertRaisesRegex(Stop,'wall clearance'):
            for f in np.linspace(0,1,321):
                r.check_clearance(r.kin.matrix(start+(target-start)*f,0),0,extra_margin=.005)
        waypoints=r.plan_empty_cartesian_return(0,target)
        a=r.kin.matrix(start,0);b=r.kin.matrix(target,0)
        rotations=Slerp([0,1],Rotation.from_matrix([a[:3,:3],b[:3,:3]]));q=start.copy()
        for f in np.linspace(0,1,321):
            q=r.solve_pose(0,a[:3,3]*(1-f)+b[:3,3]*f,rotations(f).as_matrix(),q,reference_bias=bias)
            r.check_clearance(r.kin.matrix(q,0),0,extra_margin=.005)
            for alpha in (.5,1.):r.check_clearance(r.kin.matrix(q+bias*alpha,0),0)
        self.assertTrue(np.allclose(waypoints[-1][0],b[:3,3]))
        self.assertTrue(np.allclose(waypoints[-1][1],b[:3,:3]))
        r.emit.assert_not_called()

    def test_blocked_or_loaded_return_sends_no_motion(self):
        r,_,target=self.robot();r.held=True
        with self.assertRaisesRegex(Stop,'empty hand'):r.plan_empty_cartesian_return(0,target)
        r.held=False;r.check_clearance=Mock(side_effect=Stop('blocked corridor'))
        with self.assertRaisesRegex(Stop,'blocked corridor'):r.plan_empty_cartesian_return(0,target)
        r.emit.assert_not_called()

    def test_recorded_return_feedback_keeps_limits_and_reaches_lift_pose(self):
        r,state,target=self.robot();waypoints=r.plan_empty_cartesian_return(0,target)
        bias=r.command-state;commands=[];r.low=np.zeros(16);r.high=np.ones(16)
        limits=np.vstack(r.kin.limits);r.low[JOINTS]=limits[:,0];r.high[JOINTS]=limits[:,1]
        def emit(command,**kwargs):
            self.assertLessEqual(np.max(abs(command[JOINTS]-r.command[JOINTS])),.012+1e-9)
            self.assertLessEqual(np.max(abs(command[JOINTS]-state[JOINTS])),.25+1e-9)
            for alpha in (0.,.5,1.):
                r.check_clearance(r.kin.matrix(state[:7]+(command[:7]-state[:7])*alpha,0),0)
            r.command=command.copy();state[:]+=np.clip(command-bias-state,-.006,.006)
            commands.append(command.copy())
            self.assertLess(len(commands),1600)
        r.emit=emit
        for i,(point,rot) in enumerate(waypoints):
            r.cartesian(0,point,rot,'retreat_cartesian',position_tolerance=.003,
                        orientation_tolerance=.02,settle_frames=5 if i==len(waypoints)-1 else 1)
        actual=r.kin.matrix(state[:7],0);goal=r.kin.matrix(target,0)
        self.assertLess(np.linalg.norm(actual[:3,3]-goal[:3,3]),.003)
        self.assertLess(np.linalg.norm(Rotation.from_matrix(goal[:3,:3]@actual[:3,:3].T).as_rotvec()),.02)
        self.assertTrue(commands)


if __name__=='__main__':unittest.main()
