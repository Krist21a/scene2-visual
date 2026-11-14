import json
import unittest
import numpy as np
from bootstrap import ROOT
from scene2.payload import screwdriver_payload,source_payload
from scene2.perception import masks,Detector,Part


class PayloadReplayTests(unittest.TestCase):
    def test_source_profile_points_to_metal_for_either_pca_axis_sign(self):
        for sign in (-1,1):
            part=Part('C',[-.06,0,.012],[sign,0],[.075,.02],.024,[],1.,1.,
                      [0,0,.012],[.22,.02],occupied_xy=[[-.10,0],[-.02,0]])
            center,rotation,half=source_payload(part,np.eye(4),0.)
            self.assertGreater(rotation[:,0]@(np.asarray(part.full_center)-part.center),0.)
            self.assertGreaterEqual(half[0],.111-1e-9)
            self.assertAlmostEqual(part.handle_length,.08)
            np.testing.assert_allclose(center,part.full_center)

    def test_dense_handle_does_not_trim_supported_metal_tip(self):
        s=dict(np.load(ROOT/'tests/fixtures/c-held-tip.npz'))
        evidence=json.loads((ROOT/'tests/fixtures/c-held-tip.json').read_text())
        length=np.ptp(np.asarray(evidence['source_footprint'])@evidence['source_axis'])
        tcp=np.asarray(evidence['tcp'])
        payload,info=screwdriver_payload(s,tcp,length)
        axis=np.asarray(info['axis']);center=np.asarray(info['center'])
        observed_tip=np.asarray(evidence['tip_support'])
        # This independently saved RGB-D tip cluster lay ~17 mm beyond the
        # old combined-percentile endpoint despite repeated "fit" checks.
        predicted_tip=center+axis*payload[2][0]
        self.assertGreaterEqual(float((predicted_tip-observed_tip)@axis),-.001)
        self.assertLess(float((predicted_tip-observed_tip)@axis),.004)
        self.assertGreaterEqual(info['extent'][0],length-1e-8)

    def test_source_motion_box_encloses_the_observed_full_footprint(self):
        s=dict(np.load(ROOT/'tests/fixtures/c-source-tip.npz'))
        parts,_=Detector().detect(s)
        rods=[p for p in parts if p.kind=='C']
        self.assertTrue(rods)
        for part in rods:
            axis=np.asarray(part.axis);basis=np.column_stack((axis,[-axis[1],axis[0]]))
            local=(np.asarray(part.footprint)-np.asarray(part.full_center)[:2])@basis
            self.assertTrue(np.all(abs(local)<=np.asarray(part.full_extent)/2+1e-7))

    def setUp(self):
        self.s=dict(np.load(ROOT/'tests/fixtures/c-held-head.npz'))
        self.tcp=np.array(json.loads((ROOT/'tests/fixtures/c-held-tcp.json').read_text()))

    def test_occluded_handle_keeps_source_length(self):
        payload,info=screwdriver_payload(self.s,self.tcp,.202)
        self.assertGreaterEqual(info['extent'][0],.202-1e-8)
        self.assertGreater(info['shaft_points'],50)
        self.assertLess(info['axis'][2],-.15)
        self.assertAlmostEqual(np.linalg.det(payload[1]),1.,places=8)

    def test_no_metal_shaft_cannot_establish_payload_pose(self):
        colours,_=masks(self.s['rgb'])
        self.s['rgb'][colours['A']]=[70,70,70]
        with self.assertRaises(ValueError):screwdriver_payload(self.s,self.tcp,.202)

    def test_shaft_direction_does_not_flip_with_grasp_offset(self):
        _,original=screwdriver_payload(self.s,self.tcp,.202)
        shifted=self.tcp.copy();shifted[:3,3]+=np.asarray(original['axis'])*.04
        _,moved=screwdriver_payload(self.s,shifted,.202)
        self.assertGreater(np.dot(original['axis'],moved['axis']),.98)
        self.assertGreaterEqual(moved['extent'][0],.202-1e-8)
