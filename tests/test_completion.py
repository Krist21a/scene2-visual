import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import unittest
from unittest.mock import Mock, patch
import numpy as np
from scene2.perception import Part,Bin
from scene2.task import fits_bin,Sorter
from scene2.control import Stop


class PlacementGeometryTests(unittest.TestCase):
    def setUp(self):
        self.bin=Bin('C',[.5,0],[.4,-.1],[.6,.1],0,.15,1.)

    def part(self,center,axis,extent):
        return Part('C',center,axis,extent,.03,[0,0,10,10],1.,1.,center,extent)

    def test_center_inside_is_insufficient_when_shaft_crosses_wall(self):
        p=self.part([.5,0,.015],[1,0],[.22,.02])
        self.assertTrue(self.bin.contains(p.center[:2]))
        self.assertFalse(fits_bin(p,self.bin))

    def test_diagonal_full_shaft_fits(self):
        p=self.part([.5,0,.015],[2**-.5,2**-.5],[.22,.02])
        self.assertTrue(fits_bin(p,self.bin))

    def test_wrong_bin_rejected(self):
        p=self.part([.5,.25,.015],[1,0],[.06,.03])
        self.assertFalse(fits_bin(p,self.bin))

    def test_observed_movement_changes_fit(self):
        p=self.part([.57,0,.015],[1,0],[.06,.03])
        self.assertFalse(fits_bin(p,self.bin))

    def test_return_path_sampling_preserves_placement_count(self):
        for prior in (0,1):
            with self.subTest(prior=prior):
                sorter=Sorter.__new__(Sorter)
                robot=Mock(side=0)
                robot.measured.return_value=np.zeros(16)
                robot.kin.matrix.return_value=np.eye(4)
                sorter.robot=robot
                sorter.initial_counts={'B':prior}
                sorter.completed=[]
                sorter.observer_return=None
                sorter.log=Mock()
                sorter.confirm_placed=Mock()
                part=Part('B',[.3,0,.02],[1,0],[.05,.04],.04,[0,0,10,10],1.,1.)
                source=np.array(part.center)
                target=Bin('B',[.5,0],[.4,-.1],[.6,.1],0,.15,1.)
                sorter.finish_placement(part,target,source,np.eye(3),.3,np.eye(3),
                    np.array([.5,0,.2]),np.zeros(7),np.ones(7)*.8,
                    np.array([.3,0,.3]),[np.zeros(7)])
                self.assertEqual(sorter.confirm_placed.call_args.args[3],prior)
                self.assertEqual(len(sorter.completed),1)

    def test_current_transport_checks_actual_start_before_search(self):
        sorter=Sorter.__new__(Sorter)
        sorter.bin_parts={k:[] for k in 'ABC'}
        robot=Mock(side=0,table=0.)
        robot.measured.return_value=np.arange(16)*.01
        robot.kin.matrix.return_value=np.eye(4)
        robot.check_clearance.side_effect=Stop('Hand/bin wall clearance failed')
        sorter.robot=robot
        part=Part('B',[.3,0,.02],[1,0],[.05,.04],.04,[0,0,10,10],1.,1.)
        with self.assertRaisesRegex(Stop,'wall'):
            sorter.plan_place(part,self.bin,np.eye(3),np.array(part.center),.3,from_current=True)
        robot.transfer_pose.assert_not_called()
        robot.solve_pose.assert_not_called()
        np.testing.assert_equal(robot.kin.matrix.call_args.args[0],np.arange(7)*.01)

    def test_occupied_near_edge_can_use_checked_far_edge(self):
        sorter=Sorter.__new__(Sorter)
        sorter.initial_counts={'A':1};sorter.completed=[];sorter.log=Mock()
        settled=Part('A',[.44,0,.04],[1,0],[.03,.03],.07,[],1.,1.)
        sorter.bin_parts={'A':[settled]}
        robot=Mock(side=1,table=0.)
        robot.measured.return_value=np.zeros(16)
        robot.transfer_pose.return_value=(np.array([.3,-.2,.25]),np.zeros(7))
        robot.kin.matrix.return_value=np.eye(4)
        def ik(side,goal,rotation,seed,**kwargs):
            if goal[0]<.55:raise Stop('Near edge has no reachable route')
            return np.ones(7)*.1
        robot.solve_pose.side_effect=ik;sorter.robot=robot
        part=Part('A',[.3,-.2,.01],[1,0],[.06,.03],.02,[],1.,1.)
        bin=Bin('A',[.5,0],[.4,-.1],[.6,.1],0,.15,1.)
        result=sorter.plan_place(part,bin,np.eye(3),np.array(part.center),.25,from_current=True)
        self.assertGreater(result[1][0],.55)
        self.assertGreater(robot.check_clearance.call_count,30)

    def test_second_c_prefers_a_different_diagonal(self):
        sorter=Sorter.__new__(Sorter);sorter.initial_counts={'C':1};sorter.completed=[];sorter.log=Mock()
        axis=np.array([1.,1.])/np.sqrt(2)
        sorter.bin_parts={'C':[Part('C',[.5,0,.04],axis.tolist(),[.08,.02],.08,[],1.,1.)]}
        robot=Mock(side=0,table=0.);robot.measured.return_value=np.zeros(16);robot.command=np.zeros(16)
        robot.kin.limits=np.tile([-np.pi,np.pi],(2,7,1))
        robot.kin.matrix.return_value=np.eye(4);robot.solve_pose.return_value=np.zeros(7)
        sorter.robot=robot;sorter.plan_release_roll=Mock()
        part=Part('C',[.3,0,.01],[1,0],[.08,.02],.02,[],1.,1.,[.35,0,.01],[.21,.02])
        local=np.array([[0.,1,0],[1,0,0],[0,0,-1]])
        payload=(np.zeros(3),local,np.array([.105,.012,.012]))
        # This synthetic kinematics test exercises the discrete fallback.
        # Continuous FK constraints are covered with the recorded real URDF.
        with patch('scene2.release_optimizer.release_candidates',return_value=iter(())):
            result=sorter.plan_place(part,self.bin,local,np.array(part.center),.25,payload_override=payload,from_current=True)
        release_axis=(result[0]@local)[:2,0];release_axis/=np.linalg.norm(release_axis)
        self.assertLess(abs(np.dot(release_axis,axis)),.2)


if __name__=='__main__':unittest.main()
