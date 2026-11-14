import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene2.control import Robot,Stop
from scene2.perception import Part

class GraspPriorityTests(unittest.TestCase):
    def test_coarse_route_at_next_height_precedes_fine_deep_grasp(self):
        r=Robot.__new__(Robot);r.table=0.;r.measured=lambda:np.zeros(16);r.command=np.zeros(16)
        r.log=Mock();r.joints=Mock();r.check_clearance=Mock();poses={}
        def solve(side,xyz,rotation,seed=None,**kwargs):
            q=np.zeros(7);q[0]=len(poses)+1
            pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=xyz;poses[q[0]]=pose
            return q
        r.solve_pose=solve;r.kin=Mock();r.kin.matrix.side_effect=lambda q,side:poses[q[0]]
        r.kin.limits=[np.tile([-1e6,1e6],(7,1)) for _ in range(2)]
        p=Part('A',[.3,-.3,.01],[1.,0.],[.07,.03],.02,[],0.,1.)
        nominal=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
        def validate(rotation,source):
            relative=nominal.T@rotation
            yaw=np.arctan2(relative[1,0],relative[0,0])
            if not (abs(yaw-np.deg2rad(5))<1e-6 or (abs(yaw)<1e-6 and source[2]>=.015)):
                raise Stop('No route at this heading and height')
        _,source=r.orient_for_grasp(1,p,np.array([.3,-.3,.12]),validator=validate,execute=False)
        self.assertAlmostEqual(source[2],.015)
        self.assertFalse(any(c.args[0]=='grasp_search_start' and c.kwargs['fine_yaw'] for c in r.log.call_args_list))
        r.joints.assert_not_called()

    def test_fine_heading_remains_available_after_all_coarse_routes_fail(self):
        r=Robot.__new__(Robot);r.table=0.;r.command=np.zeros(16);r.measured=lambda:np.zeros(16)
        r.log=Mock();r.joints=Mock();r.check_clearance=Mock();poses={}
        nominal=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
        def solve(side,xyz,rotation,seed=None,**kwargs):
            relative=nominal.T@rotation
            if abs(np.arctan2(relative[1,0],relative[0,0])-np.deg2rad(5))>1e-6:
                raise Stop('Only a fine heading reaches this fixture')
            q=np.zeros(7);q[0]=len(poses)+1
            pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=xyz;poses[q[0]]=pose
            return q
        r.solve_pose=solve;r.kin=Mock();r.kin.matrix.side_effect=lambda q,side:poses[q[0]]
        r.kin.limits=[np.tile([-1e6,1e6],(7,1)) for _ in range(2)]
        p=Part('A',[.3,-.3,.01],[1.,0.],[.07,.03],.02,[],0.,1.)
        _,point=r.orient_for_grasp(1,p,np.array([.3,-.3,.12]),execute=False)
        self.assertAlmostEqual(point[2],.014)
        self.assertTrue(any(c.args[0]=='grasp_search_start' and c.kwargs['fine_yaw'] for c in r.log.call_args_list))
        r.joints.assert_not_called()

    def test_a_tries_higher_shallow_grasp_before_steeper_grasp(self):
        r=Robot.__new__(Robot);r.table=0.;r.measured=lambda:np.zeros(16)
        r.command=np.zeros(16)
        r.log=Mock();r.joints=Mock();r.transfer_pose=Mock()
        poses={}
        def solve(side,xyz,rotation,seed=None,**kwargs):
            q=np.zeros(7);q[0]=len(poses)+1
            mat=np.eye(4);mat[:3,:3]=rotation;mat[:3,3]=xyz;poses[q[0]]=mat
            return q
        r.solve_pose=solve;r.kin=Mock();r.kin.matrix.side_effect=lambda q,side:poses[q[0]]
        r.kin.limits=[np.tile([-1e6,1e6],(7,1)) for _ in range(2)]
        def clearance(mat,side,**kwargs):
            # Both a steep low grasp and a shallow raised grasp are feasible.
            if mat[2,3]<.02 and abs(mat[2,1])<.4:raise Stop('table reserve')
        r.check_clearance=clearance
        p=Part('A',[.3,-.3,.01],[1.,0.],[.07,.03],.02,[],0.,1.)
        rotation,source=r.orient_for_grasp(1,p,np.array([.3,-.3,.12]))
        self.assertLess(abs(rotation[2,1]),.27)
        self.assertGreaterEqual(source[2],.02)
        r.joints.assert_called_once()

    def test_occluded_second_a_still_blocks_fingers_in_recorded_shallow_pose(self):
        import json
        from bootstrap import ROOT
        from scene1.kinematics import UrdfArmKinematics
        from scene2.perception import Bin
        data=json.loads((ROOT/'tests/fixtures/a-shallow-grasp.json').read_text())
        r=Robot.__new__(Robot);r.table=data['table'];r.side=1;r.stage='replay'
        r.command=np.array(data['command']);r.measured=lambda:np.array(data['measured'])
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.log=Mock();r.joints=Mock();r.held=False;r.payload=None
        part=Part(**data['part']);r.target=part
        r.parts=[Part(**p) for p in data['parts']]
        r.parts=[part if p.kind=='A' and np.linalg.norm(np.array(p.center)-part.center)<.025 else p for p in r.parts]
        if not any(p is part for p in r.parts):r.parts.append(part)
        r.bins={k:Bin(**v) for k,v in data['bins'].items()}
        from scipy.spatial.transform import Rotation
        axis=np.array(part.axis)
        nominal=np.array([[axis[1],axis[0],0],[-axis[0],axis[1],0],[0,0,1]])
        rotation=nominal@Rotation.from_euler('z',np.pi/2).as_matrix()@Rotation.from_euler('x',-.13).as_matrix()
        source=np.array(part.center);source[2]=max(source[2]+.004,r.table+.013)+.001
        pose=np.eye(4);pose[:3,3]=source;pose[:3,:3]=rotation
        with self.assertRaisesRegex(Stop,'Hand/part clearance failed'):
            r.check_clearance(pose,1,extra_margin=.002,table_reserve=.003)
        r.joints.assert_not_called()
