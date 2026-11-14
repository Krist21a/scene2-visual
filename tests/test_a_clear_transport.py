import json
import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin
from scene2.task import Sorter


class ClearTransportTests(unittest.TestCase):
    def test_recorded_deeper_a_grasp_can_turn_after_vertical_lift_with_occupied_bin(self):
        data=json.loads((ROOT/'tests/fixtures/a-clear-lift.json').read_text())
        r=Robot.__new__(Robot);r.side=1;r.table=data['table'];r.stage='replay'
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.measured=lambda:np.array(data['measured']);r.command=np.array(data['command'])
        r.log=Mock();r.held=False;r.payload=None
        part=Part(**data['part']);r.target=part
        r.parts=[Part(**p) for p in data['parts']]
        r.parts=[p for p in r.parts if p.kind!='A' or np.linalg.norm(np.array(p.center)-part.center)>=.025]+[part]
        r.bins={k:Bin(**v) for k,v in data['bins'].items()}
        r.transfer_pose=Mock(side_effect=Stop('The grasp attitude cannot reach the old high endpoint'))
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=dict.fromkeys('ABC',0)
        s.completed=[dict(kind='A')]
        s.bin_parts={k:[Part(**p) for p in v] for k,v in data['settled'].items()}
        self.assertEqual(len(s.bin_parts['A']),1)
        source=np.array(data['source']);rotation=np.array(data['rotation'])
        self.assertLessEqual(source[2]-part.center[2],.009001)
        q=r.solve_pose(1,source,rotation)
        r.check_clearance(r.kin.matrix(q,1),1,extra_margin=.002,table_reserve=.003)
        result=s.plan_place(part,r.bins['A'],rotation,source,.3)
        _,_,_,start,point,_,route=result
        self.assertAlmostEqual(point[2],r.table+.10)
        r.transfer_pose.assert_not_called()
        axis=np.array(part.axis)
        object_rotation=np.column_stack((np.r_[axis,0],np.r_[[-axis[1],axis[0]],0],[0,0,1]))
        payload=(rotation.T@(np.array(part.full_center)-source),rotation.T@object_rotation,
                 np.r_[part.full_extent,part.top-r.table]/2)
        # A denser independent sweep covers both payload and open-hand retreat.
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,81):
                r.check_clearance(r.kin.matrix(a+(b-a)*fraction,1),1,payload=payload,
                                  extra_margin=.010,gripper_open=True)


if __name__=='__main__':unittest.main()
