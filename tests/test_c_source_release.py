import json
import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.payload import source_payload
from scene2.perception import Part,Bin
from scene2.release import free_drop_separations,side_opening_clear
from scene2.settling import c_profile,held_fits,settling_fits
from scene2.task import Sorter,C_RELEASE_MARGIN,C_RELEASE_RESERVE


class SourceReleaseTests(unittest.TestCase):
    def check_source_release(self,fixture,occupied=False):
        d=json.loads((ROOT/'tests/fixtures'/fixture).read_text())
        r=Robot.__new__(Robot);r.side=0;r.table=d['table'];r.stage='replay';r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.command=np.array(d['grasp_command']);r.measured=lambda:np.array(d['grasp_measured'])
        r.parts=[Part(**p) for p in d['parts']];r.bins={k:Bin(**v) for k,v in d['bins'].items()}
        r.target=part=Part(**d['part']);r.held=False;r.payload=None
        r.emit=Mock(side_effect=AssertionError('Read-only grasp search emitted a command'))
        r.joints=Mock(side_effect=AssertionError('Read-only grasp search moved the robot'))
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=d['initial_counts']
        s.completed=d['completed'];s.bin_parts={k:[Part(**p) for p in v] for k,v in d['settled'].items()}
        r.settled_parts=[p for parts in s.bin_parts.values() for p in parts]
        s.release_bins={k:Bin(**b) for k,b in d['release_bins'].items()}
        if 'rejected_low_route' in d:
            old=d['rejected_low_route'];grasp=np.eye(4)
            grasp[:3,:3]=old['rotation'];grasp[:3,3]=old['source']
            old_payload=source_payload(part,grasp,r.table)
            with self.assertRaisesRegex(Stop,'wall clearance'):
                for a,b in zip([old['start']]+old['route'][:-1],old['route']):
                    for fraction in np.linspace(0,1,321):
                        r.check_clearance(r.kin.matrix(np.asarray(a)+(np.asarray(b)-a)*fraction,0),0,
                                          payload=old_payload,extra_margin=.010)
        if occupied or d.get('cavity_snapshot'):
            snapshot=dict(np.load(ROOT/'tests/fixtures'/d.get('cavity_snapshot','c-cavity-occlusion.npz')))
            s.release_empty_cavity_reference=(Bin(**d['empty_cavity']),{k:Bin(**v) for k,v in d['empty_bins'].items()})
            s.release_bins['C']=s.tracked_release_cavity(snapshot,r.bins)
            self.assertTrue(any(c.args[0]=='release_cavity_registered' for c in s.log.call_args_list))
            if occupied:self.assertEqual(len(s.bin_parts['C']),1)
        checked=[]
        def validate(rotation,source):
            checked.append(s.plan_place(part,r.bins['C'],rotation,source,.2))
        hover=np.array(part.center);hover[2]=r.table+.10
        rotation,source=r.orient_for_grasp(0,part,hover,validator=validate,execute=False)
        if fixture=='c-source-low-start-c36.json':
            self.assertEqual({k:len(v) for k,v in s.bin_parts.items()},dict(A=2,B=2,C=0))
            self.assertGreaterEqual(np.linalg.norm(source[:2]-np.asarray(part.center)[:2]),.010-1e-8)
            self.assertAlmostEqual(checked[-1][4][2],r.table+.10)
            self.assertLess(np.linalg.norm(r.kin.matrix(checked[-1][3],0)[:3,3]-checked[-1][4]),.005)
        if 'failed_grasp' in d:
            minimum=.010-1e-8 if fixture=='c-source-release-grasp-c20.json' else d.get('min_grasp_change',.010)
            self.assertGreaterEqual(np.linalg.norm(source[:2]-np.asarray(d['failed_grasp'])[:2]),minimum)
        if fixture=='c-source-release-grasp-c20.json':
            observed_center=np.asarray(part.full_center)[:2]
            # A 20mm offset shortens the observed lever by 9.999mm rather
            # than the prior 15mm; retain the 10mm goal with 10um tolerance.
            self.assertLessEqual(np.linalg.norm(observed_center-source[:2]),
                                 np.linalg.norm(observed_center-np.asarray(d['failed_grasp'])[:2])-.010+1e-5)
        grasp=np.eye(4);grasp[:3,:3]=rotation;grasp[:3,3]=source
        payload=source_payload(part,grasp,r.table)
        _,_,q,start,_,_,route=checked[-1]
        pose=r.kin.matrix(q,0);object_rotation=pose[:3,:3]@payload[1]
        self.assertTrue(side_opening_clear(pose))
        center=pose[:3,3]+pose[:3,:3]@payload[0];cavity=s.release_bins['C']
        self.assertTrue(held_fits(cavity,center,abs(object_rotation)@payload[2],C_RELEASE_MARGIN+C_RELEASE_RESERVE,
                                 (c_profile(payload[2],part.handle_length)@object_rotation.T)[:,:2]))
        self.assertTrue(settling_fits(cavity,center,object_rotation[:,0],payload[2],
                                    margin=.003+C_RELEASE_RESERVE,handle_length=part.handle_length))
        self.assertGreaterEqual(free_drop_separations(pose,payload,part.handle_length,r.table).min(),.002-1e-7)
        if fixture=='c-source-reserve-c25.json':
            self.assertGreaterEqual(free_drop_separations(pose,payload,part.handle_length,r.table).min(),.003-1e-7)
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,321 if fixture=='c-source-low-start-c36.json' else 81):
                r.check_clearance(r.kin.matrix(a+(b-a)*fraction,0),0,payload=payload,extra_margin=.010)
        r.emit.assert_not_called();r.joints.assert_not_called()

    def test_full_profile_selects_releasable_grasp_before_closing(self):
        self.check_source_release('c-source-release-grasp.json')

    def test_second_source_has_checked_route_with_occupied_bin_and_fresh_cavity(self):
        self.check_source_release('c-source-release-occupied.json',occupied=True)

    def test_pivot_failure_source_has_checked_alternative_grasp(self):
        self.check_source_release('c-source-release-grasp-c11.json')

    def test_current_registered_dimensions_keep_c13_source_route_reachable(self):
        self.check_source_release('c-source-release-grasp-c13.json')

    def test_pivoting_second_c_has_more_balanced_checked_grasp(self):
        self.check_source_release('c-source-release-grasp-c20.json')

    def test_second_c_source_reserves_room_before_in_hand_pivot(self):
        self.check_source_release('c-source-reserve-c25.json')

    def test_full_round_uses_checked_low_start_and_shorter_lever_grasp(self):
        self.check_source_release('c-source-low-start-c36.json')
