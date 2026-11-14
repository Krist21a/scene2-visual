import json
import unittest
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene1.motion import JOINTS
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin


class ReleaseAlignmentTests(unittest.TestCase):
    def test_position_arrival_does_not_hide_release_rotation_error(self):
        robot=Robot.__new__(Robot)
        robot.command=np.zeros(16);robot.low=np.full(16,-2.);robot.high=np.full(16,2.)
        robot.table=-.1;robot.log=Mock();robot.measured=lambda:robot.command.copy()
        robot.kin=Mock();commands=[]
        def feedback(q,side):
            pose=np.eye(4)
            pose[:3,:3]=Rotation.from_rotvec([0,0,.04 if len(commands)<6 else .002]).as_matrix()
            return pose,np.eye(6,7)
        robot.kin.matrix_and_jacobian.side_effect=feedback
        def emit(command,**kwargs):
            self.assertLessEqual(np.max(abs(command-robot.command)),.012+1e-12)
            commands.append(command.copy());robot.command=command.copy()
        robot.emit=emit
        robot.cartesian(0,np.zeros(3),np.eye(3),'release_align',position_tolerance=.0015,orientation_tolerance=.005)
        self.assertGreaterEqual(len(commands),10)
        self.assertLess(np.linalg.norm(robot.log.call_args.kwargs['error'][3:]),.005)

    def test_rotation_limit_cannot_be_relaxed(self):
        robot=Robot.__new__(Robot)
        with self.assertRaises(ValueError):
            robot.cartesian(0,np.zeros(3),np.eye(3),'release_align',orientation_tolerance=.2)

    def test_recorded_endpoint_uses_remaining_joints_when_one_is_saturated(self):
        # This is a constrained feedback model using recorded state and the
        # public URDF, not a claim that a physical release has succeeded.
        d=json.loads((ROOT/'tests/fixtures/c-cartesian-endpoint-c18.json').read_text())
        r=Robot.__new__(Robot)
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.command=np.array(d['command']);state=np.array(d['measured']);initial=state.copy()
        bias=r.command-state;r.measured=lambda:state.copy();r.side=0;r.table=d['table'];r.log=Mock()
        r.held=True;r.payload=tuple(np.array(v) for v in d['payload']);r.payload_shaft_sign=1.
        r.target=Part(**d['part']);r.parts=[Part(**p) for p in d['parts']]
        r.bins={k:Bin(**b) for k,b in d['bins'].items()}
        r.settled_parts=[Part(**p) for ps in d['settled'].values() for p in ps]
        r.low=np.zeros(16);r.high=np.ones(16);limits=np.vstack(r.kin.limits)
        r.low[JOINTS]=limits[:,0];r.high[JOINTS]=limits[:,1];commands=[]
        def emit(command,**kwargs):
            self.assertLessEqual(np.max(abs(command-r.command)),.012+1e-9)
            self.assertLessEqual(np.max(abs(command[JOINTS]-state[JOINTS])),.25+1e-9)
            for alpha in (0.,.5,1.):
                r.check_clearance(r.kin.matrix(state[:7]+(command[:7]-state[:7])*alpha,0),0,payload=r.payload)
            r.command=command.copy();commands.append(command.copy())
            state[:]+=np.clip(command-bias-state,-.006,.006)
            state[2]=initial[2]
            if len(commands)>500:self.fail('Alignment stalled with one saturated joint')
        r.emit=emit
        r.cartesian(0,d['goal'],d['rotation'],'release_align',position_tolerance=.0015,orientation_tolerance=.005)
        pose=r.kin.matrix(state[:7],0)
        self.assertLess(np.linalg.norm(np.array(d['goal'])-pose[:3,3]),.0015)
        self.assertLess(np.linalg.norm(Rotation.from_matrix(np.array(d['rotation'])@pose[:3,:3].T).as_rotvec()),.005)
        self.assertEqual(state[2],initial[2])
        self.assertGreater(np.max(abs(state-initial)),.01)
        self.assertTrue(commands)

    def test_reference_outside_compensation_window_cannot_move_further_outward(self):
        r=Robot.__new__(Robot);r.command=np.zeros(16);r.command[0]=.24
        r.low=np.full(16,-2.);r.high=np.full(16,2.);r.table=-.1;r.log=Mock()
        r.measured=lambda:np.zeros(16);r.kin=Mock();jacobian=np.eye(6,7);jacobian[0,6]=1.
        r.kin.matrix_and_jacobian.return_value=(np.eye(4),jacobian)
        sent=[]
        def emit(command,**kwargs):
            sent.append(command.copy());raise Stop('Captured first bounded correction')
        r.emit=emit
        with self.assertRaisesRegex(Stop,'Captured'):
            r.cartesian(0,[.01,.01,0],np.eye(3),'release_align')
        self.assertLessEqual(sent[0][0],.24)
        self.assertGreater(sent[0][6],0.)
