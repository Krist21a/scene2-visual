import unittest
from bootstrap import ROOT
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scene2.release import free_drop_clear, free_drop_separations


class FreeReleaseTests(unittest.TestCase):
    def test_optimiser_separation_agrees_with_independent_prism_guard(self):
        rng=np.random.default_rng(17)
        evidence=json.loads((ROOT/'tests/fixtures/c-release-shifted.json').read_text())
        payload=tuple(np.asarray(value) for value in evidence['payload'])
        for rotation in Rotation.random(80,random_state=rng).as_matrix():
            pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=[.54,.44,.3]
            gaps=free_drop_separations(pose,payload,evidence['part']['extent'][0],evidence['table'])
            self.assertEqual(bool(np.all(gaps>0)),
                             free_drop_clear(pose,payload,evidence['part']['extent'][0],evidence['table']))

    def test_recorded_rim_failure_has_lower_finger_in_drop_path(self):
        evidence=json.loads((ROOT/'tests/fixtures/c-release-rim.json').read_text())
        measured=evidence['payload']
        payload=tuple(np.asarray(measured[k]) for k in ('local_center','local_rotation','half'))
        self.assertFalse(free_drop_clear(np.array(measured['tcp']),payload,evidence['handle_length'],evidence['table']))

    def test_side_opening_allows_vertical_fall(self):
        pose=np.eye(4);pose[2,3]=.3
        payload=(np.zeros(3),Rotation.from_euler('z',np.pi/2).as_matrix(),np.array([.1,.012,.012]))
        self.assertTrue(free_drop_clear(pose,payload,.08,0.))

    def test_lower_finger_blocks_fall_even_without_initial_contact(self):
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('y',np.pi/2).as_matrix();pose[2,3]=.3
        payload=(np.zeros(3),Rotation.from_euler('z',np.pi/2).as_matrix(),np.array([.1,.012,.012]))
        self.assertFalse(free_drop_clear(pose,payload,.08,0.))

    def test_wide_handle_cannot_pass_open_jaws(self):
        pose=np.eye(4);pose[2,3]=.3
        payload=(np.array([0.,.06,0.]),Rotation.from_euler('z',np.pi/2).as_matrix(),np.array([.1,.060,.012]))
        self.assertFalse(free_drop_clear(pose,payload,.08,0.))
