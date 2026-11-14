import json
import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot, Stop
from scene2.perception import Part, Bin
from scipy.spatial.transform import Rotation,Slerp


class GraspReferenceTests(unittest.TestCase):
    def robot(self, fixture='a-descend-reference-a23.json'):
        d=json.loads((ROOT/'tests/fixtures'/fixture).read_text())
        r=Robot.__new__(Robot);r.side=1;r.table=d['table'];r.stage='hover';r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.low=np.r_[r.kin.limits[0][:,0],0.,r.kin.limits[1][:,0],0.]
        r.high=np.r_[r.kin.limits[0][:,1],1.,r.kin.limits[1][:,1],1.]
        pre=d.get('pre',d)
        state=np.array(pre['measured']);r.command=np.array(pre['command'])
        r.measured=lambda:state.copy();r.held=False;r.payload=None
        r.target=Part(**d['part']);r.parts=[Part(**p) for p in d['parts']
            if p['kind']!='A' or np.linalg.norm(np.array(p['center'])-r.target.center)>.03]+[r.target]
        r.bins={k:Bin(**v) for k,v in d['bins'].items()};r.settled_parts=[]
        return r,d,state

    def test_recorded_nominal_grasp_is_rejected_before_descent(self):
        r,d,state=self.robot();old=d['grasp']
        q=r.solve_pose(1,old['grasp_point'],np.array(old['rotation']),state[8:15])
        r.check_clearance(r.kin.matrix(q,1),1,extra_margin=.002,table_reserve=.003)
        with self.assertRaisesRegex(Stop,'Hand/part'):
            r.check_grasp_reference(1,q,r.command[8:15]-state[8:15],.002,.003)

    def test_alternative_grasp_retains_both_nominal_and_reference_clearance(self):
        r,d,state=self.robot();new=d['alternative']
        q=r.solve_pose(1,new['source'],np.array(new['rotation']),state[8:15])
        bias=r.command[8:15]-state[8:15]
        r.check_grasp_reference(1,q,bias,.002,.003)
        # Densely audit the full nominal/reference interval independently.
        for fraction in np.linspace(0,1,31):
            r.check_clearance(r.kin.matrix(q+bias*fraction,1),1,extra_margin=.002,table_reserve=.003)

    def test_reference_outside_joint_limit_is_not_clipped_into_acceptance(self):
        r,d,state=self.robot();q=state[8:15];bias=np.zeros(7)
        bias[0]=r.kin.limits[1][0,1]-q[0]+.01
        with self.assertRaisesRegex(Stop,'joint limits'):
            r.check_grasp_reference(1,q,bias,.002,.003)

    def test_bias_bounded_ik_preserves_deeper_shallow_grasp(self):
        r,d,state=self.robot('a-bounded-grasp-a24.json')
        r.joints=Mock();hover=np.array(r.target.center);hover[2]=r.table+.12
        rotation,source=r.orient_for_grasp(1,r.target,hover,execute=False)
        self.assertLessEqual(source[2],d['source'][2]+.001)
        self.assertLess(abs(rotation[2,1]),.27)
        bias=r.command[8:15]-state[8:15]
        q=r.solve_pose(1,source,rotation,state[8:15],reference_bias=bias)
        r.check_grasp_reference(1,q,bias,.002,.003)
        r.joints.assert_not_called()

    def test_changing_descent_bias_has_small_engaged_height_correction(self):
        r,d,_=self.robot('a-descend-feedback-a27.json')
        state=np.array(d['early']['measured']);r.command=np.array(d['early']['command'])
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        bias=r.command[8:15]-state[8:15]
        q=r.solve_pose(1,source,rotation,state[8:15],reference_bias=bias)
        with self.assertRaisesRegex(Stop,'table'):
            r.check_grasp_reference(1,q,bias,.002,.003)
        checked=r.refined_grasp_height(1,source,rotation,state,source[2])
        self.assertGreater(checked[2],source[2])
        self.assertLessEqual(checked[2],min(source[2]+.003,r.target.top-.001))
        np.testing.assert_allclose(checked[:2],source[:2])
        q=r.solve_pose(1,checked,rotation,state[8:15],reference_bias=bias)
        for fraction in np.linspace(0,1,31):
            r.check_clearance(r.kin.matrix(q+bias*fraction,1),1,extra_margin=.002,table_reserve=.003)

    def test_late_unsafe_height_is_rejected_without_expanding_depth(self):
        r,d,_=self.robot('a-descend-feedback-a27.json')
        state=np.array(d['late']['measured']);r.command=np.array(d['late']['command'])
        r.measured=lambda:state.copy();r.emit=Mock();r.gripper=Mock()
        source=np.array(d['grasp']['grasp_point'])
        with self.assertRaisesRegex(Stop,'No engaged A grasp height'):
            r.refined_grasp_height(1,source,np.array(d['grasp']['rotation']),state,source[2])
        r.emit.assert_not_called();r.gripper.assert_not_called()

    def test_height_refinement_never_returns_below_already_selected_height(self):
        r,d,_=self.robot('a-descend-feedback-a27.json')
        state=np.array(d['early']['measured']);r.command=np.array(d['early']['command'])
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        minimum=source[2]+.002
        checked=r.refined_grasp_height(1,source,rotation,state,minimum)
        self.assertGreaterEqual(checked[2],minimum-1e-9)

    def test_recorded_three_retreat_failure_has_engaged_four_mm_correction(self):
        r,d,_=self.robot('a-engaged-height-a31.json');r.emit=Mock();r.gripper=Mock()
        source=np.array(d['grasp']['grasp_point'])
        for feedback,rotation in ((d['before_first_retreat'],d['grasp']['rotation']),
                                  (d['pre'],d['final_rotation'])):
            with self.subTest(stage=feedback.get('stage','final')):
                state=np.array(feedback['measured']);r.command=np.array(feedback['command'])
                checked=r.refined_grasp_height(1,source,np.array(rotation),state,source[2]+.003)
                self.assertAlmostEqual(checked[2]-source[2],.004)
                self.assertLessEqual(checked[2],r.target.top-.001)
                np.testing.assert_allclose(checked[:2],source[:2])
                bias=r.command[8:15]-state[8:15]
                q=r.solve_pose(1,checked,np.array(rotation),state[8:15],reference_bias=bias)
                for fraction in np.linspace(0,1,61):
                    r.check_clearance(r.kin.matrix(q+bias*fraction,1),1,
                                      extra_margin=.002,table_reserve=.003)
        r.emit.assert_not_called();r.gripper.assert_not_called()

    def test_additional_height_cannot_cross_recorded_engagement_ceiling(self):
        r,d,state=self.robot('a-engaged-height-a31.json');r.emit=Mock();r.gripper=Mock()
        source=np.array(d['grasp']['grasp_point'])
        self.assertGreater(source[2]+.005,r.target.top-.001)
        with self.assertRaisesRegex(Stop,'No engaged A grasp height'):
            r.refined_grasp_height(1,source,np.array(d['final_rotation']),state,source[2]+.005)
        with self.assertRaisesRegex(Stop,'No checked local A grasp'):
            r.refined_grasp_pose(1,source,np.array(d['grasp']['rotation']),state,source[2]+.005)
        r.emit.assert_not_called();r.gripper.assert_not_called()

    def test_recorded_drifting_descent_converges_with_lagged_checked_ik_steps(self):
        r,d,state=self.robot('a-convergence-a32.json');r.emit=Mock();r.gripper=Mock()
        source=np.array(d['grasp']['grasp_point'])
        point=np.array(d['refined']['goal']);rotation=np.array(d['refined']['rotation'])
        physical_bias=r.command[8:15]-state[8:15]
        stable=0
        for iteration in range(40):
            checked=r.refined_grasp_height(1,source,rotation,state,point[2])
            step=r.grasp_ik_step(1,checked,rotation,state)
            self.assertLessEqual(np.max(np.abs(step)),.012+1e-12)
            r.command[8:15]+=step
            self.assertLessEqual(np.max(np.abs(r.command[8:15]-state[8:15])),.22)
            for fraction in np.linspace(0,1,17):
                q=state[8:15]+(r.command[8:15]-state[8:15])*fraction
                r.check_clearance(r.kin.matrix(q,1),1)
            # A fixed load bias with first-order actuator lag, not a physics
            # success claim. Every reference still passes the real geometry.
            state[8:15]+=.35*(r.command[8:15]-physical_bias-state[8:15])
            pose=r.kin.matrix(state[8:15],1);r.check_clearance(pose,1)
            arrived=(np.linalg.norm(checked-pose[:3,3])<.002 and
                     np.linalg.norm(Rotation.from_matrix(rotation@pose[:3,:3].T).as_rotvec())<.02 and
                     pose[2,3]<=r.target.top-.001)
            stable=stable+1 if arrived else 0
            if stable>=5:break
        self.assertEqual(stable,5)
        r.emit.assert_not_called();r.gripper.assert_not_called()

    def test_ik_correction_cannot_use_the_recorded_unsafe_original_depth(self):
        r,d,state=self.robot('a-convergence-a32.json');r.emit=Mock();r.gripper=Mock()
        with self.assertRaises(Stop):
            r.grasp_ik_step(1,np.array(d['grasp']['grasp_point']),
                           np.array(d['refined']['rotation']),state)
        r.emit.assert_not_called();r.gripper.assert_not_called()

    def test_shifted_grip_survives_recorded_bias_that_rejects_old_refinement(self):
        r,d,state=self.robot('a-failed-bias-refinement-a33.json');r.emit=Mock()
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        current=r.command[8:15]-state[8:15]
        failures=[np.array(d[k]['command'])[8:15]-np.array(d[k]['measured'])[8:15]
                  for k in ('failed','later_failure')]
        old=d['old_refinement']
        q=r.solve_pose(1,old['goal'],np.array(old['rotation']),state[8:15],reference_bias=current)
        r.check_grasp_reference(1,q,current,.002,.003)
        with self.assertRaisesRegex(Stop,'Hand/part'):
            r.check_grasp_reference(1,q,failures[-1],.002,.003)
        point,rot=r.refined_grasp_pose(1,source,rotation,state,old['goal'][2],[rotation],
                                      reference_biases=failures)
        offset=np.linalg.norm(point[:2]-source[:2])
        self.assertGreater(offset,.001);self.assertLessEqual(offset,.006+1e-9)
        self.assertLessEqual(point[2],r.target.top-.001)
        actual=r.kin.matrix(state[8:15],1)
        orientations=Slerp([0.,1.],Rotation.from_matrix(np.array([actual[:3,:3],rot])))
        q=state[8:15].copy()
        for fraction in np.linspace(0,1,61):
            xyz=actual[:3,3]*(1-fraction)+point*fraction
            q=r.solve_pose(1,xyz,orientations(fraction).as_matrix(),q,reference_bias=current)
            r.check_clearance(r.kin.matrix(q,1),1)
            r.check_grasp_reference(1,q,current,0.,0.)
        q=r.solve_pose(1,point,rot,state[8:15],reference_bias=current)
        for failed in failures:
            for blend in np.linspace(0,1,5):
                bias=current*(1-blend)+failed*blend
                for fraction in np.linspace(0,1,31):
                    r.check_clearance(r.kin.matrix(q+bias*fraction,1),1,
                                      extra_margin=.002,table_reserve=.003)
        r.emit.assert_not_called()

    def test_failed_reference_history_must_be_finite_and_within_tracking_limit(self):
        r,d,state=self.robot('a-failed-bias-refinement-a33.json');r.emit=Mock()
        for bias in (np.full(7,np.nan),np.full(7,.251),np.zeros(6)):
            with self.subTest(bias=bias),self.assertRaisesRegex(Stop,'Invalid failed-descent'):
                r.refined_grasp_pose(1,np.array(d['grasp']['grasp_point']),
                    np.array(d['grasp']['rotation']),state,d['old_refinement']['goal'][2],
                    reference_biases=[bias])
        r.emit.assert_not_called()

    def test_descent_keeps_shifted_xy_but_original_depth_and_search_anchor(self):
        r,d,state=self.robot('a-failed-bias-refinement-a33.json');r.emit=Mock()
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        point=np.array(d['point']);rot=np.array(d['rotation'])
        near=r.solve_pose(1,point,rot,state[8:15],reference_bias=r.command[8:15]-state[8:15])
        def refined(*args,**kwargs):
            state[8:15]=near
            return point,rot
        r.refined_grasp_height=Mock(side_effect=[Stop('Recorded failed reference'),RuntimeError('probe complete')])
        r.retreat_for_grasp_refinement=Mock();r.refined_grasp_pose=Mock(side_effect=refined)
        with self.assertRaisesRegex(RuntimeError,'probe complete'):
            r.finish_grasp_descent(1,source,rotation)
        active=r.refined_grasp_height.call_args_list[1].args[1]
        np.testing.assert_allclose(active[:2],point[:2]);self.assertEqual(active[2],source[2])
        np.testing.assert_allclose(r.refined_grasp_pose.call_args.args[1],source)
        self.assertEqual(len(r.refined_grasp_pose.call_args.kwargs['reference_biases']),1)
        r.emit.assert_not_called()

    def test_recorded_transient_endpoint_failure_still_allows_checked_transit(self):
        r,d,state=self.robot('a-transient-approach-a34.json')
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        point=np.array(d['refined']['goal']);rot=np.array(d['refined']['rotation'])
        self.assertGreater(np.linalg.norm(point-r.pose(1)[:3,3]),.009)
        active=source.copy();active[:2]=point[:2]
        with self.assertRaisesRegex(Stop,'No engaged A grasp height'):
            r.refined_grasp_height(1,active,rot,state,point[2])
        r.refined_grasp_height=Mock(side_effect=[Stop('initial recovery'),AssertionError('premature endpoint check')])
        r.retreat_for_grasp_refinement=Mock();r.refined_grasp_pose=Mock(return_value=(point,rot))
        r.emit=Mock(side_effect=RuntimeError('checked transit reached'))
        with self.assertRaisesRegex(RuntimeError,'checked transit reached'):
            r.finish_grasp_descent(1,source,rotation)
        self.assertEqual(r.refined_grasp_height.call_count,1)
        command=r.emit.call_args.args[0]
        self.assertLessEqual(np.max(np.abs(command[8:15]-r.command[8:15])),.012+1e-12)
        for fraction in np.linspace(0,1,31):
            q=state[8:15]+fraction*(command[8:15]-state[8:15])
            r.check_clearance(r.kin.matrix(q,1),1)

    def test_recovery_rechecks_terminal_geometry_before_accepting_near_pose(self):
        r,d,state=self.robot('a-transient-approach-a34.json');r.emit=Mock()
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        point=np.array(d['refined']['goal']);rot=np.array(d['refined']['rotation'])
        near=r.solve_pose(1,point,rot,state[8:15])
        def refined(*args,**kwargs):
            state[8:15]=near
            return point,rot
        r.refined_grasp_height=Mock(side_effect=[Stop('initial recovery'),RuntimeError('terminal geometry rechecked')])
        r.retreat_for_grasp_refinement=Mock();r.refined_grasp_pose=Mock(side_effect=refined)
        with self.assertRaisesRegex(RuntimeError,'terminal geometry rechecked'):
            r.finish_grasp_descent(1,source,rotation)
        self.assertEqual(r.refined_grasp_height.call_count,2)
        r.emit.assert_not_called()

    def test_recorded_successful_nominal_grasp_near_top_is_preserved(self):
        r,d,state=self.robot('a-descend-success-a27.json')
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        self.assertGreater(source[2],r.target.top-.001)
        np.testing.assert_allclose(r.refined_grasp_height(1,source,rotation,state,source[2]),source)

    def test_recorded_near_grasp_has_bounded_checked_attitude_correction(self):
        r,d,state=self.robot('a-local-refinement-a29.json');r.emit=Mock()
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        point,rot=r.refined_grasp_pose(1,source,rotation,state,source[2]+.003)
        np.testing.assert_allclose(point[:2],source[:2])
        self.assertAlmostEqual(point[2]-source[2],.003)
        self.assertLess(point[2],r.target.top-.001)
        self.assertLessEqual(np.linalg.norm(Rotation.from_matrix(rot@rotation.T).as_rotvec()),.10+1e-9)
        bias=r.command[8:15]-state[8:15];actual=r.kin.matrix(state[8:15],1)
        orientations=Slerp([0.,1.],Rotation.from_matrix(np.array([actual[:3,:3],rot])))
        seed=state[8:15].copy()
        for fraction in np.linspace(0,1,61):
            pose=np.eye(4);pose[:3,:3]=orientations(fraction).as_matrix()
            pose[:3,3]=actual[:3,3]*(1-fraction)+point*fraction
            r.check_clearance(pose,1)
            seed=r.solve_pose(1,pose[:3,3],pose[:3,:3],seed,reference_bias=bias)
            r.check_grasp_reference(1,seed,bias,0.,0.)
        r.check_clearance(r.kin.matrix(seed,1),1,extra_margin=.002,table_reserve=.003)
        r.check_grasp_reference(1,seed,bias,.002,.003)
        r.emit.assert_not_called()

    def test_recorded_refinement_can_retreat_before_search(self):
        r,d,state=self.robot('a-local-refinement-a29.json');r.cartesian=Mock();r.emit=Mock()
        before=r.pose(1)
        r.retreat_for_grasp_refinement(1)
        call=r.cartesian.call_args
        np.testing.assert_allclose(call.args[1],before[:3,3]+[0,0,.012])
        np.testing.assert_allclose(call.args[2],before[:3,:3])
        self.assertEqual(call.args[3],'grasp_refine_retreat')
        r.emit.assert_not_called()

    def test_rejected_attitude_is_not_repeated_in_local_search(self):
        r,d,state=self.robot('a-local-refinement-a29.json')
        source=np.array(d['grasp']['grasp_point']);rotation=np.array(d['grasp']['rotation'])
        _,first=r.refined_grasp_pose(1,source,rotation,state,source[2]+.003)
        _,second=r.refined_grasp_pose(1,source,rotation,state,source[2]+.003,[first])
        self.assertGreaterEqual(np.linalg.norm(Rotation.from_matrix(second@first.T).as_rotvec()),.012)

    def test_explicit_fine_heading_fallback_retains_a_checked_route(self):
        from scene2.task import Sorter
        from scene2.payload import source_payload
        r,d,state=self.robot('a-fine-grasp-a30.json');r.emit=Mock();r.joints=Mock()
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=dict.fromkeys('ABC',0)
        s.completed=d['completed'];s.bin_parts={k:[Part(**p) for p in v] for k,v in d['settled'].items()}
        s.release_bins=dict(r.bins);routes=[]
        def validate(rotation,point):
            routes.append(s.plan_place(r.target,r.bins['A'],rotation,point,max(b.rim for b in r.bins.values())+.055))
        hover=np.array(r.target.center);hover[2]=r.table+.12
        rotation,point=r.orient_for_grasp(1,r.target,hover,validator=validate,execute=False,a_yaw_refinement=True)
        self.assertLessEqual(point[2],d['grasp']['grasp_point'][2]-.004)
        pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=point
        payload=source_payload(r.target,pose,r.table)
        _,_,_,start,_,_,route=routes[-1]
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,61):
                r.check_clearance(r.kin.matrix(a+(b-a)*fraction,1),1,payload=payload,extra_margin=.010)
        r.emit.assert_not_called();r.joints.assert_not_called()

    def test_recorded_fine_flat_failure_prefers_checked_coarse_tilted_route(self):
        from scene2.task import Sorter
        from scene2.payload import source_payload
        r,d,state=self.robot('a-coarse-priority-a35.json');r.emit=Mock();r.joints=Mock()
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=dict.fromkeys('ABC',0)
        s.completed=[];s.bin_parts={k:[] for k in 'ABC'};s.release_bins=dict(r.bins);routes=[]
        def validate(rotation,point):
            routes.append(s.plan_place(r.target,r.bins['A'],rotation,point,max(b.rim for b in r.bins.values())+.055))
        hover=np.array(r.target.center);hover[2]=r.table+.12
        rotation,point=r.orient_for_grasp(1,r.target,hover,validator=validate,execute=False)
        self.assertGreater(point[2]-d['grasp']['grasp_point'][2],.0009)
        self.assertGreater(abs(rotation[2,1]),.10)
        self.assertFalse(any(c.args[0]=='grasp_search_start' and c.kwargs['fine_yaw'] for c in r.log.call_args_list))
        pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=point
        payload=source_payload(r.target,pose,r.table)
        _,_,_,start,_,_,route=routes[-1]
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,61):
                r.check_clearance(r.kin.matrix(a+(b-a)*fraction,1),1,payload=payload,extra_margin=.010)
        r.emit.assert_not_called();r.joints.assert_not_called()


if __name__=='__main__':unittest.main()
