import unittest
import numpy as np
from scene2.perception import Bin


class CavityPolygonTests(unittest.TestCase):
    def test_rotated_corners_and_original_margin(self):
        angle=.2
        rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        corners=np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])@rotation.T
        bin=Bin('C',[0,0],[-.08,-.08],[.08,.08],0,.15,0,corners.tolist())
        self.assertTrue(bin.contains(np.array([.09,.09])@rotation.T,.003))
        self.assertFalse(bin.contains(np.array([.098,0])@rotation.T,.003))
        self.assertFalse(bin.contains(np.array([.101,0])@rotation.T))

