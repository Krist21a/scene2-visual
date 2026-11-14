"""C21 opened from a static-clear pose but the part landed across the rim."""
import json
import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin
from scene2.release import free_drop_clear,side_opening_clear,C_OPENING_PLANNING_LIMIT
from scene2.settling import c_profile,held_fits,settling_fits
from scene2.task import Sorter,C_RELEASE_MARGIN,C_RELEASE_RESERVE


class OpeningTests(unittest.TestCase):
    def replay(self):
        d=json.loads((ROOT/'tests/fixtures/c-opening-c21.json').read_text())
        r=Robot.__new__(Robot);r.side=0;r.table=d['table'];r.stage='replay';r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.command=np.array(d['command']);r.measured=lambda:np.array(d['measured'])
        r.payload=tuple(np.array(v) for v in d['payload']);r.payload_shaft_sign=1.;r.held=True
        r.target=Part(**d['part']);r.parts=[Part(**p) for p in d['parts']]
        r.bins={k:Bin(**b) for k,b in d['bins'].items()}
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=d['initial_counts'];s.completed=d['completed']
        s.release_bins={k:Bin(**b) for k,b in d['release_bins'].items()}
        s.bin_parts={k:[Part(**p) for p in v] for k,v in d['settled'].items()}
        r.settled_parts=[p for ps in s.bin_parts.values() for p in ps]
        s.release_seeds=[np.array(q) for q in d['release_seeds']]
        return r,s

    def test_recorded_steep_opening_replans_with_real_occupancy(self):
        r,s=self.replay();old=r.pose(0)
        self.assertTrue(free_drop_clear(old,r.payload,r.target.handle_length,r.table))
        self.assertFalse(side_opening_clear(old))
        self.assertEqual(len(s.bin_parts['C']),1)
        rot,goal,q,start,_,_,route=s.plan_place(r.target,r.bins['C'],old[:3,:3],old[:3,3],old[2,3],
                                              payload_override=r.payload,from_current=True)
        pose=r.kin.matrix(q,0)
        self.assertLessEqual(abs(pose[2,0]),C_OPENING_PLANNING_LIMIT+1e-7)
        self.assertTrue(side_opening_clear(pose))
        self.assertTrue(free_drop_clear(pose,r.payload,r.target.handle_length,r.table))
        center=goal+rot@r.payload[0];rotation=rot@r.payload[1];half=r.payload[2]
        self.assertTrue(held_fits(s.release_bins['C'],center,abs(rotation)@half,C_RELEASE_MARGIN+C_RELEASE_RESERVE,
                                 (c_profile(half,r.target.handle_length)@rotation.T)[:,:2]))
        self.assertTrue(settling_fits(s.release_bins['C'],center,rotation[:,0],half,
                                     margin=.003+C_RELEASE_RESERVE,handle_length=r.target.handle_length))
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,81):
                nominal=a+(b-a)*fraction
                r.check_clearance(r.kin.matrix(nominal,0),0,payload=r.payload,extra_margin=.010)
                reference=np.clip(nominal+r.command[:7]-start,r.kin.limits[0][:,0]+.001,r.kin.limits[0][:,1]-.001)
                for alpha in (.5,1.):
                    r.check_clearance(r.kin.matrix(nominal+(reference-nominal)*alpha,0),0,payload=r.payload)

    def test_feedback_that_keeps_steep_opening_never_releases(self):
        r,s=self.replay();pose=r.pose(0);q=r.measured()[:7]
        s.measure_payload=Mock(return_value=r.payload)
        s.plan_place=Mock(return_value=(pose[:3,:3],pose[:3,3],q,q,pose[:3,3],[],[]))
        r.cartesian=Mock();r.joint_path=Mock();r.emit=Mock();r.gripper=Mock()
        with self.assertRaisesRegex(Stop,'three corrections'):
            s.finish_placement(r.target,r.bins['C'],pose[:3,3],pose[:3,:3],pose[2,3],
                               pose[:3,:3],pose[:3,3],q,q,pose[:3,3],[])
        r.gripper.assert_not_called();self.assertTrue(r.held)
        rejected=[c.kwargs for c in s.log.call_args_list if c.args[0]=='release_payload_rejected']
        self.assertEqual(len(rejected),4)
        self.assertTrue(all(c['held_inside'] and c['settles_inside'] and c['free_drop_clear'] for c in rejected))
        self.assertTrue(all(not c['opening_clear'] for c in rejected))


if __name__=='__main__':unittest.main()
