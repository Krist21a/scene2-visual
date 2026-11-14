import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import unittest
import numpy as np
from scene2.control import Robot,Stop,prism_overlap,columns_overlap
from scene2.perception import Bin
from types import SimpleNamespace


class PayloadClearanceTests(unittest.TestCase):
    def robot(self):
        r=Robot.__new__(Robot)
        r.command=np.zeros(16);r.table=0.;r.bins={};r.parts=[]
        r.held=True;r.side=1;r.target=None;r.stage='transport';r.log=lambda *a,**kw:None
        r.payload=(np.zeros(3),np.eye(3),np.array([.03,.02,.03]))
        return r

    def test_payload_below_fingers_still_blocks_table(self):
        r=self.robot();mat=np.eye(4);mat[2,3]=.02
        with self.assertRaisesRegex(Stop,'table'):r.check_clearance(mat,1)

    def test_clear_carried_volume_allowed(self):
        r=self.robot();mat=np.eye(4);mat[2,3]=.25
        r.check_clearance(mat,1)

    def test_table_reserve_rejects_marginal_height(self):
        r=self.robot();mat=np.eye(4);mat[2,3]=.034
        r.check_clearance(mat,1)
        with self.assertRaisesRegex(Stop,'table'):
            r.check_clearance(mat,1,table_reserve=.003)
        mat[2,3]+=.003
        r.check_clearance(mat,1,table_reserve=.003)

    def test_long_shaft_blocks_wall_even_when_tcp_is_clear(self):
        r=self.robot();r.payload=(np.array([.10,0,0]),np.eye(3),np.array([.1,.01,.01]))
        r.bins={'C':Bin('C',[.5,0],[.4,-.1],[.6,.1],.01,.15,1)}
        mat=np.eye(4);mat[:3,3]=[.2,0,.10]
        with self.assertRaisesRegex(Stop,'wall'):r.check_clearance(mat,1)

    def test_thin_shaft_can_clear_wall_where_handle_would_not(self):
        r=self.robot();r.target=SimpleNamespace(kind='C',handle_length=.08)
        r.payload=(np.array([.10,0,0]),np.eye(3),np.array([.1,.012,.012]))
        r.bins={'B':Bin('B',[.5,0],[.4,-.1],[.6,.1],.01,.15,1)}
        mat=np.eye(4);mat[:3,3]=[.2,.137,.10]
        r.check_clearance(mat,1)
        r.payload_shaft_sign=-1.
        with self.assertRaisesRegex(Stop,'wall'):r.check_clearance(mat,1)

    def test_segmented_shaft_still_rejects_actual_wall_contact(self):
        r=self.robot();r.target=SimpleNamespace(kind='C',handle_length=.08)
        r.payload=(np.array([.10,0,0]),np.eye(3),np.array([.1,.012,.012]))
        r.bins={'B':Bin('B',[.5,0],[.4,-.1],[.6,.1],.01,.15,1)}
        mat=np.eye(4);mat[:3,3]=[.2,.12,.10]
        with self.assertRaisesRegex(Stop,'wall'):r.check_clearance(mat,1)

    def settled_robot(self,footprint=True):
        r=self.robot()
        r.bins={'C':Bin('C',[.5,0],[.3,-.2],[.7,.2],0,.15,1)}
        r.settled_parts=[SimpleNamespace(kind='C',center=[.5,0,.025],
            full_center=None,axis=[1,0],extent=[.12,.04],full_extent=None,
            top=.05,footprint=[[.44,-.02],[.56,-.02],[.56,.02],[.44,.02]] if footprint else None)]
        return r

    def test_transport_and_open_hand_clear_above_settled_part(self):
        r=self.settled_robot();mat=np.eye(4);mat[:3,3]=[.5,0,.25]
        for opened in (False,True):r.check_clearance(mat,1,gripper_open=opened)

    def test_carried_object_cannot_hit_settled_part(self):
        mat=np.eye(4);mat[:3,3]=[.5,0,.065]
        for footprint in (False,True):
            with self.subTest(footprint=footprint):
                r=self.settled_robot(footprint)
                with self.assertRaisesRegex(Stop,'part'):r.check_clearance(mat,1)

    def test_empty_retreat_hand_cannot_hit_settled_part(self):
        r=self.settled_robot();r.held=False
        mat=np.eye(4);mat[:3,3]=[.5,0,.015]
        with self.assertRaisesRegex(Stop,'part'):r.check_clearance(mat,1,gripper_open=False)


class ObservedFootprintTests(unittest.TestCase):
    def test_recorded_narrow_grasp_requires_tracking_reserve(self):
        import json
        from scene2.perception import Part
        data=json.loads((Path(__file__).parent/'fixtures/a-narrow-grasp.json').read_text())
        r=Robot.__new__(Robot);r.command=np.zeros(16);r.table=data['table'];r.bins={}
        r.target=Part(**data['part']);r.parts=[r.target];r.side=1;r.held=False;r.payload=None
        r.stage='descend';r.log=lambda *a,**kw:None
        mat=np.eye(4);mat[:3,:3]=data['rotation'];mat[:3,3]=data['point']
        r.check_clearance(mat,1)
        with self.assertRaisesRegex(Stop,'part'):r.check_clearance(mat,1,extra_margin=.002)

    def test_empty_corner_of_bounding_rectangle_is_clear(self):
        footprint=[[0,0],[1,0],[0,1]]
        self.assertFalse(prism_overlap(np.array([.8,.8,.05]),np.eye(3),np.array([.05,.05,.02]),footprint,0,.1))

    def test_slanted_face_contact_is_blocked(self):
        footprint=[[0,0],[1,0],[0,1]]
        self.assertTrue(prism_overlap(np.array([.5,.5,.05]),np.eye(3),np.array([.05,.05,.02]),footprint,0,.1))

    def test_vertical_separation_is_clear(self):
        self.assertFalse(prism_overlap(np.array([.2,.2,.2]),np.eye(3),np.array([.05,.05,.02]),[[0,0],[1,0],[0,1]],0,.1))

    def test_observed_hole_and_solid_bar_are_distinguished(self):
        xy=np.array([[x,y] for x in np.linspace(-.04,.04,41) for y in (-.03,.03)])
        self.assertFalse(columns_overlap(np.array([0,0,.01]),np.eye(3),np.array([.01,.01,.005]),xy,0,.02))
        self.assertTrue(columns_overlap(np.array([0,.025,.01]),np.eye(3),np.array([.01,.01,.005]),xy,0,.02))


if __name__=='__main__':unittest.main()
