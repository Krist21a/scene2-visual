import unittest
import numpy as np
from scene2.perception import Bin
from scene2.settling import settling_offset,settled_corners,release_center,settling_fits,held_fits,c_profile,support_bias

class SettlingTests(unittest.TestCase):
    def test_release_reserve_keeps_original_margins_under_small_translation_error(self):
        box=Bin('C',[0,0],[-.12,-.12],[.12,.12],0,.15,0)
        axis=np.array([.8,0.,-.6]);half=np.array([.1,.012,.012]);extent=np.array([.092,.012,.08])
        boundary=release_center(box,box,axis,half,extent,.010)
        self.assertFalse(held_fits(box,boundary+[.0015,0],extent,.010))
        robust=release_center(box,box,axis,half,extent,.010,reserve=.002)
        for delta in ([.0015,0],[-.0015,0],[0,.0015],[0,-.0015]):
            center=robust+delta
            self.assertTrue(held_fits(box,center,extent,.010))
            self.assertTrue(settling_fits(box,np.r_[center,0],axis,half))

    def test_recorded_second_c_needs_new_attitude_for_nonzero_execution_reserve(self):
        import json
        from bootstrap import ROOT
        data=json.loads((ROOT/'tests/fixtures/c-release-margin.json').read_text())
        box=Bin(**data['cavity'])
        for observation in data['measurements']:
            pose=np.array(observation['tcp'])
            rotation=pose[:3,:3]@observation['local_rotation'];half=np.array(observation['half'])
            extent=abs(rotation)@half;axis=rotation[:,0]
            boundary=release_center(box,box,axis,half,extent,.010,data['handle_length'],rotation)
            self.assertTrue(held_fits(box,boundary,extent,.010,(c_profile(half,data['handle_length'])@rotation.T)[:,:2]))
            with self.assertRaises(ValueError):
                release_center(box,box,axis,half,extent,.010,data['handle_length'],rotation,reserve=.002)

    def test_negative_release_reserve_cannot_reduce_clearance(self):
        box=Bin('C',[0,0],[-.12,-.12],[.12,.12],0,.15,0)
        with self.assertRaises(ValueError):
            release_center(box,box,[.8,0,-.6],[.1,.012,.012],[.092,.012,.08],.010,reserve=-.001)

    def test_requested_settling_buffer_cannot_reduce_original_margin(self):
        box=Bin('C',[0,0],[-.12,-.12],[.12,.12],0,.15,0)
        with self.assertRaisesRegex(ValueError,'Settling margin'):
            release_center(box,box,[.8,0,-.6],[.1,.012,.012],[.092,.012,.08],.010,settling_margin=.002)

    def test_occupied_compensation_and_prediction_use_same_bias(self):
        box=Bin('C',[0,0],[-.3,-.3],[.3,.3],0,.15,0)
        axis=np.array([.6,0,-.8]);half=np.array([.1,.012,.012]);extent=np.array([.07,.012,.09])
        empty=release_center(box,box,axis,half,extent,.01)
        bias=support_bias(box,True)
        occupied=release_center(box,box,axis,half,extent,.01,bias=bias)
        np.testing.assert_allclose(occupied,empty-bias)
        np.testing.assert_allclose(settled_corners(np.r_[occupied,0],axis,half,bias=bias),
                                   settled_corners(np.r_[empty,0],axis,half))
        np.testing.assert_array_equal(support_bias(box,False),[0,0])

    def test_profile_preserves_wide_handle_and_thin_shaft(self):
        cavity=Bin('C',[0,0],[-.12,-.02],[.12,.02],0,.15,0,
                   cavity=[[-.12,-.02],[.12,-.007],[.12,.007],[-.12,.02]])
        half=np.array([.1,.012,.012])
        self.assertFalse(held_fits(cavity,[0,0],half,.001))
        self.assertTrue(held_fits(cavity,[0,0],half,.001,c_profile(half,.08)[:,:2]))
        wide=np.array([.1,.02,.02])
        self.assertFalse(held_fits(cavity,[0,0],wide,.001,c_profile(wide,.08)[:,:2]))

    def test_low_tip_pivot_keeps_low_tip_fixed(self):
        axis=np.array([.6,0.,-.8]);center=np.array([.5,0.,.2]);half=np.array([.1,.01,.01])
        offset,direction=settling_offset(axis,half[0])
        np.testing.assert_allclose(center[:2]+offset+direction*half[0],center[:2]+axis[:2]*half[0])
        inverse,_=settling_offset(-axis,half[0])
        np.testing.assert_allclose(inverse,offset)

    def test_suspended_fit_does_not_imply_settled_fit(self):
        b=Bin('C',[.5,0],[.4,-.12],[.6,.12],0,.15,0)
        axis=np.array([.6,0.,-.8]);half=np.array([.09,.01,.01]);center=np.array([.5,0,.2])
        self.assertTrue(b.contains(center[:2]-[.07,.01],.01))
        self.assertTrue(b.contains(center[:2]+[.07,.01],.01))
        self.assertFalse(settling_fits(b,center,axis,half))

    def test_compensation_is_clipped_and_still_requires_full_fit(self):
        b=Bin('C',[.5,0],[.38,-.14],[.62,.14],0,.15,0)
        axis=np.array([.6,0.,-.8]);half=np.array([.09,.01,.01]);extent=np.array([.062,.01,.08])
        xy=release_center(b,b,axis,half,extent,.01)
        self.assertGreater(xy[0],.5)
        self.assertTrue(b.contains(xy+extent[:2],.01))
        self.assertTrue(settling_fits(b,np.r_[xy,.2],axis,half))

    def test_vertical_rod_is_not_accepted(self):
        b=Bin('C',[.5,0],[.3,-.2],[.7,.2],0,.15,0)
        self.assertFalse(settling_fits(b,[.5,0,.2],[0,0,1],[.1,.01,.01]))

    def test_diagonal_translation_recovers_region_missed_by_xy_clipping(self):
        visible=Bin('C',[.55,0],[.15,-.9],[.95,.9],0,.15,0)
        cavity=Bin('C',[0,0],[-1,-1],[1,1],0,.15,0,cavity=[[0,-1],[1,0],[0,1],[-1,0]])
        axis=np.array([.6,.6,-np.sqrt(.28)]);half=np.array([.5,.01,.01]);extent=np.array([.32,.32,.3])
        offset,_=settling_offset(axis,half[0])
        clipped=np.clip(-offset,np.asarray(visible.lower)+extent[:2]+.01,np.asarray(visible.upper)-extent[:2]-.01)
        self.assertFalse(settling_fits(cavity,np.r_[clipped,0],axis,half))
        center=release_center(visible,cavity,axis,half,extent,.01)
        self.assertTrue(settling_fits(cavity,np.r_[center,0],axis,half))
        self.assertTrue(visible.contains(center-extent[:2],.01))
        self.assertTrue(visible.contains(center+extent[:2],.01))

