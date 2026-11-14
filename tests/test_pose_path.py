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
from scene2.task import C_RELEASE_MARGIN
from scene2.settling import c_profile,held_fits,settling_fits
from scene2.release import free_drop_clear,side_opening_clear


class PosePathTests(unittest.TestCase):
    def test_recorded_correction_tracks_pose_with_saturated_joint(self):
        # Recorded geometry and feedback, with joint 3 deliberately held fixed.
        # This tests controller feasibility, not physical placement success.
        d=json.loads((ROOT/'tests/fixtures/c-pose-path-c19.json').read_text())
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
            for side,start in ((0,0),(1,8)):
                for alpha in (0.,.5,1.):
                    r.check_clearance(r.kin.matrix(state[start:start+7]+(command[start:start+7]-state[start:start+7])*alpha,side),side)
            r.command=command.copy();commands.append(command.copy())
            state[:]+=np.clip(command-bias-state,-.006,.006)
            state[2]=initial[2]
            if len(commands)>500:
                pose=r.kin.matrix(state[:7],0)
                error=np.r_[np.array(d['goal'])-pose[:3,3],
                            Rotation.from_matrix(np.array(d['rotation'])@pose[:3,:3].T).as_rotvec()]
                self.fail(f'Pose path stalled: error={error.tolist()}, bias={(r.command-state)[:7].tolist()}')
        r.emit=emit
        self.assertGreater(abs(np.array(d['goal_q14'])[2]-state[2]),.06)
        # The old correction is outside the compensated command range. Do
        # not accept an approximate IK and then wait for an impossible pose.
        from scene2.task import RELEASE_JOINT_MARGIN
        with self.assertRaisesRegex(Stop,'No bounded IK'):
            r.solve_pose(0,np.array(d['goal']),np.array(d['rotation']),state[:7],
                         joint_margin=RELEASE_JOINT_MARGIN,reference_bias=bias[:7])
        # Preserve the historical feedback-tracking case independently of
        # release planning. Its steep jaw opening is now rejected; following
        # this route successfully must never imply permission to release.
        recorded=d['historical_controller_route']
        self.assertFalse(recorded['release_authorized'])
        rotation=np.array(recorded['rotation']);goal=np.array(recorded['goal'])
        q=np.array(recorded['q']);route=[np.array(v) for v in recorded['route']]
        r.stage='replay'
        d['goal']=goal;d['rotation']=rotation
        self.assertLess(abs(q[2]-state[2]),.00011)
        self.assertTrue(np.all(q+bias[:7]<r.high[:7]-.0099))
        self.assertTrue(np.all(q+bias[:7]>r.low[:7]+.0099))
        target=state[JOINTS].copy()
        for waypoint in route:
            target[:7]=waypoint
            r.joint_path(target,'payload_release_correction',settle=False,pose_tracking=True)
        self.assertEqual(r.log.call_args.args[0],'joint_path_streamed')
        r.cartesian(0,d['goal'],d['rotation'],'release_align',position_tolerance=.0015,orientation_tolerance=.005)
        pose=r.kin.matrix(state[:7],0)
        self.assertLess(np.linalg.norm(np.array(d['goal'])-pose[:3,3]),.0015)
        self.assertLess(np.linalg.norm(Rotation.from_matrix(np.array(d['rotation'])@pose[:3,:3].T).as_rotvec()),.005)
        self.assertEqual(state[2],initial[2])
        center=pose[:3,3]+pose[:3,:3]@r.payload[0];rotation=pose[:3,:3]@r.payload[1]
        cavity=Bin(**d['release_bins']['C'])
        self.assertTrue(held_fits(cavity,center,abs(rotation)@r.payload[2],C_RELEASE_MARGIN,
                                  (c_profile(r.payload[2],r.target.handle_length)@rotation.T)[:,:2]))
        self.assertTrue(settling_fits(cavity,center,rotation[:,0],r.payload[2],handle_length=r.target.handle_length))
        self.assertTrue(free_drop_clear(pose,r.payload,r.target.handle_length,r.table))
        self.assertFalse(side_opening_clear(pose))
        self.assertTrue(commands)

    def mock_robot(self):
        r=Robot.__new__(Robot);r.command=np.zeros(16);r.measured=lambda:np.zeros(16)
        r.kin=Mock();r.kin.matrix.return_value=np.eye(4);r.log=Mock()
        r.check_clearance=Mock();r.emit=Mock();r.cartesian=Mock();r.joints=Mock()
        return r

    def test_blocked_pose_path_sends_nothing(self):
        r=self.mock_robot();r.check_clearance.side_effect=Stop('blocked')
        with self.assertRaisesRegex(Stop,'blocked'):
            r.joint_path(np.ones(14)*.3,'test',settle=False,pose_tracking=True)
        r.emit.assert_not_called()

    def test_zero_length_pose_path_requires_no_motion(self):
        r=self.mock_robot();r.joint_path(np.zeros(14),'test',pose_tracking=True)
        r.emit.assert_not_called();r.cartesian.assert_not_called();r.joints.assert_not_called()


if __name__=='__main__':unittest.main()
