import json
import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin
from scene2.release import free_drop_clear,side_opening_clear
from scene2.settling import c_profile,held_fits,settling_fits
from scene2.task import Sorter,C_RELEASE_MARGIN,C_RELEASE_RESERVE,C_ELBOW_TRANSIT_MARGIN,C_ELBOW_IK_MARGIN,C_OCCUPIED_SETTLING_MARGIN


class OccupiedReleaseMarginTests(unittest.TestCase):
    def check_recorded_release(self,fixture,reject_opening=False):
        d=json.loads((ROOT/'tests/fixtures'/fixture).read_text())
        r=Robot.__new__(Robot);r.side=0;r.table=d['table'];r.stage='replay';r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.command=np.array(d['command']);r.measured=lambda:np.array(d['measured'])
        r.payload=tuple(np.array(v) for v in d['payload']);r.payload_shaft_sign=1.;r.held=True
        part=Part(**d['part']);r.target=part;r.parts=[Part(**p) for p in d['parts']]
        r.bins={k:Bin(**b) for k,b in d['bins'].items()}
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=d['initial_counts'];s.completed=d['completed']
        s.release_bins={k:Bin(**b) for k,b in d['release_bins'].items()}
        s.bin_parts={k:[Part(**p) for p in v] for k,v in d['settled'].items()}
        s.release_seeds=[np.asarray(q) for q in d.get('release_seeds',[])]
        self.assertEqual(len(s.bin_parts['C']),d.get('expected_occupied_c',1))
        pose=np.array(d['tcp'])
        if fixture=='c-settling-reserve-c39.json':
            observed_rotation=pose[:3,:3]@r.payload[1]
            observed_center=pose[:3,3]+pose[:3,:3]@r.payload[0]
            self.assertTrue(settling_fits(s.release_bins['C'],observed_center,observed_rotation[:,0],r.payload[2],
                                         handle_length=part.handle_length))
            self.assertFalse(settling_fits(s.release_bins['C'],observed_center,observed_rotation[:,0],r.payload[2],
                                          margin=C_OCCUPIED_SETTLING_MARGIN,handle_length=part.handle_length))
        if reject_opening:
            # These historical grasps had static-clear releases with steep
            # jaw opening. The additional transient-opening guard can make
            # them infeasible; the correct result is retention, not release.
            r.emit=Mock();r.gripper=Mock()
            with self.assertRaisesRegex(Stop,'No reachable collision-checked release'):
                s.plan_place(part,r.bins['C'],pose[:3,:3],pose[:3,3],pose[2,3],
                             payload_override=r.payload,from_current=True)
            r.emit.assert_not_called();r.gripper.assert_not_called()
            self.assertTrue(r.held)
            return
        result=s.plan_place(part,r.bins['C'],pose[:3,:3],pose[:3,3],pose[2,3],payload_override=r.payload,from_current=True)
        rotation,goal,q,start,_,_,route=result
        if fixture=='c-clear-transport-c14.json':
            self.assertGreaterEqual(len(route),2)
            self.assertTrue(any(c.args[0]=='transport_clearance_route' for c in s.log.call_args_list))
        center=goal+rotation@r.payload[0];object_rotation=rotation@r.payload[1]
        extent=abs(object_rotation)@r.payload[2]
        offsets=(c_profile(r.payload[2],part.handle_length)@object_rotation.T)[:,:2]
        self.assertTrue(held_fits(s.release_bins['C'],center,extent,C_RELEASE_MARGIN+C_RELEASE_RESERVE-1e-8,offsets))
        self.assertTrue(settling_fits(s.release_bins['C'],center,object_rotation[:,0],r.payload[2],
                                    margin=.003+C_RELEASE_RESERVE-1e-8,handle_length=part.handle_length))
        if fixture=='c-settling-reserve-c39.json':
            self.assertTrue(settling_fits(s.release_bins['C'],center,object_rotation[:,0],r.payload[2],
                margin=C_OCCUPIED_SETTLING_MARGIN+C_RELEASE_RESERVE-1e-8,handle_length=part.handle_length))
        # C43's new 7mm hard floor had no route. Optimising the available room
        # must improve on the original 5mm floor, while still checking the
        # complete transport and compensation sweep independently below.
        improved_margin={'c-release-reserve-c43.json':.0065,'c-release-gauge.json':.0054}.get(fixture)
        if improved_margin is not None:
            self.assertTrue(settling_fits(s.release_bins['C'],center,object_rotation[:,0],r.payload[2],
                margin=improved_margin,handle_length=part.handle_length))
        if fixture=='c-release-held-margin-c47.json':
            # Full run C47 was repeatedly at the held boundary. More held
            # room must not consume the existing 7mm landing allowance.
            self.assertTrue(held_fits(s.release_bins['C'],center,extent,.0126,offsets))
            self.assertTrue(settling_fits(s.release_bins['C'],center,object_rotation[:,0],r.payload[2],
                                         margin=.007,handle_length=part.handle_length))
        nominal=np.eye(4);nominal[:3,:3]=rotation;nominal[:3,3]=goal
        self.assertTrue(side_opening_clear(nominal))
        self.assertTrue(side_opening_clear(r.kin.matrix(q,0)))
        self.assertTrue(free_drop_clear(nominal,r.payload,part.handle_length,r.table))
        self.assertTrue(free_drop_clear(r.kin.matrix(q,0),r.payload,part.handle_length,r.table))
        if fixture in ('c-release-high.json','c-release-gauge.json','c-release-empty-reference.json'):
            from scene2.release import free_drop_separations
            selected=[c.kwargs['drop_reserve'] for c in s.log.call_args_list if c.args[0]=='continuous_release_plan'][-1]
            self.assertIn(selected,(.002,.0003))
            self.assertGreaterEqual(free_drop_separations(r.kin.matrix(q,0),r.payload,part.handle_length,r.table).min(),selected-1e-7)
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,321 if fixture in ('c-release-reserve-c43.json','c-settling-reserve-c39.json','c-release-held-margin-c47.json') else 81):
                nominal=a+(b-a)*fraction
                if nominal[1]>1.2:
                    self.assertGreaterEqual(nominal[2]-r.kin.limits[0][2,0],C_ELBOW_TRANSIT_MARGIN-1e-8)
                r.check_clearance(r.kin.matrix(nominal,0),0,payload=r.payload,extra_margin=.010)
                reference=np.clip(nominal+r.command[:7]-start,r.kin.limits[0][:,0]+.001,r.kin.limits[0][:,1]-.001)
                for alpha in (.5,1.):
                    r.check_clearance(r.kin.matrix(nominal+(reference-nominal)*alpha,0),0,payload=r.payload)
        if fixture=='c-clear-transport-c15.json':
            # The old nominal-only route was collision-free but the measured
            # compensation brought its command reference into the B bin wall.
            for a,b in zip([start]+[np.array(q) for q in d['rejected_route'][:-1]],d['rejected_route']):
                for fraction in np.linspace(0,1,81):
                    nominal=a+(np.array(b)-a)*fraction
                    r.check_clearance(r.kin.matrix(nominal,0),0,payload=r.payload,extra_margin=.010)
            with self.assertRaises(Stop):
                for a,b in zip([start]+[np.array(q) for q in d['rejected_route'][:-1]],d['rejected_route']):
                    for fraction in np.linspace(0,1,81):
                        nominal=a+(np.array(b)-a)*fraction
                        r.check_clearance(r.kin.matrix(nominal+r.command[:7]-start,0),0,payload=r.payload)
        if fixture=='c-second-transport-c16.json':
            self.assertEqual(len(s.bin_parts['C']),1)
            self.assertTrue(any(q[1]>1.2 and q[2]-r.kin.limits[0][2,0]<.01 for q in d['rejected_route']))
            for waypoint in route[:-1]:
                self.assertGreaterEqual(waypoint[2]-r.kin.limits[0][2,0],C_ELBOW_IK_MARGIN-1e-8)

    def test_old_second_c_without_side_opening_route_is_retained(self):
        self.check_recorded_release('c-occupied-release.json',reject_opening=True)

    def test_old_high_payload_without_side_opening_route_is_retained(self):
        self.check_recorded_release('c-release-high.json',reject_opening=True)

    def test_pivoted_payload_with_complete_handle_rejects_unsafe_correction(self):
        # The old percentile handle length omitted observed thick material.
        # This recorded grasp cannot satisfy even the smaller extra fall
        # reserve once the complete handle is used; it must be held for reset.
        from scene2.release_optimizer import release_candidates
        from scene2.task import RELEASE_JOINT_MARGIN
        d=json.loads((ROOT/'tests/fixtures/c-release-shifted.json').read_text())
        part=Part(**d['part']);payload=tuple(np.asarray(v) for v in d['payload'])
        kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        self.assertGreater(part.handle_length,part.extent[0]+.001)
        candidates=list(release_candidates(kin,0,np.asarray(d['measured'])[:7],[],payload,
            part.handle_length,Bin(**d['release_bins']['C']),d['bins']['C']['rim'],d['table'],
            RELEASE_JOINT_MARGIN,C_RELEASE_MARGIN,C_RELEASE_RESERVE,
            [Part(**p) for p in d['settled']['C']],drop_reserve=.0003))
        self.assertEqual(candidates,[])

    def test_latest_drop_guard_failure_has_checked_correction(self):
        self.check_recorded_release('c-release-gauge.json')

    def test_old_occupied_payload_without_side_opening_route_is_retained(self):
        self.check_recorded_release('c-release-empty-reference.json',reject_opening=True)

    def test_old_full_tip_payload_without_side_opening_route_is_retained(self):
        self.check_recorded_release('c-release-full-tip.json',reject_opening=True)

    def test_actual_clear_lift_has_checked_route_without_fixed_high_attitude(self):
        self.check_recorded_release('c-clear-transport.json')

    def test_neighbouring_wall_requires_separate_initial_clearance_leg(self):
        self.check_recorded_release('c-clear-transport-c14.json')

    def test_feedback_compensation_reference_also_clears_neighbouring_wall(self):
        self.check_recorded_release('c-clear-transport-c15.json')

    def test_second_c_transport_avoids_loaded_elbow_limit_with_real_occupancy(self):
        self.check_recorded_release('c-second-transport-c16.json')

    def test_folded_second_c_has_checked_side_opening_route(self):
        self.check_recorded_release('c-second-transport-c17.json')

    def test_c39_second_rod_reserves_extra_landing_clearance_before_release(self):
        self.check_recorded_release('c-settling-reserve-c39.json')

    def test_c43_second_rod_improves_landing_room_with_checked_transport(self):
        self.check_recorded_release('c-release-reserve-c43.json')

    def test_c47_second_rod_adds_held_room_without_losing_landing_room(self):
        self.check_recorded_release('c-release-held-margin-c47.json')


if __name__=='__main__':unittest.main()
