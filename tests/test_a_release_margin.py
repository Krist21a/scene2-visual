import json
import unittest
from unittest.mock import Mock
import numpy as np
from bootstrap import ROOT
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot
from scene2.perception import Part, Bin, Detector
from scene2.task import Sorter, fits_bin


class BracketReleaseMarginTests(unittest.TestCase):
    def test_recorded_wall_contact_remains_rejected(self):
        snapshot=dict(np.load(ROOT/'tests/fixtures/a-wall-settling-a46.npz'))
        parts,bins=Detector().detect(snapshot,True)
        observed=[p for p in parts if p.kind=='A' and bins['A'].contains(p.center[:2])]
        self.assertEqual(len(observed),1)
        self.assertFalse(fits_bin(observed[0],bins['A']))
        # This is a physical margin failure, not missing shape recognition.
        self.assertLess(np.min(np.asarray(observed[0].footprint)-bins['A'].lower),.003)

    def test_recorded_transport_has_more_lateral_landing_room(self):
        d=json.loads((ROOT/'tests/fixtures/a-release-margin-a46.json').read_text())
        r=Robot.__new__(Robot);r.side=1;r.table=d['table'];r.stage='replay';r.log=Mock()
        r.kin=UrdfArmKinematics(description=(ROOT/'tests/fixtures/scene2-robot.urdf').read_text())
        r.command=np.asarray(d['command']);r.measured=lambda:np.asarray(d['measured'])
        r.payload=tuple(np.asarray(v) for v in d['payload']);r.held=True
        r.target=Part(**d['part']);r.parts=[Part(**p) for p in d['parts']]
        r.bins={k:Bin(**b) for k,b in d['bins'].items()}
        s=Sorter.__new__(Sorter);s.robot=r;s.log=Mock();s.initial_counts=dict.fromkeys('ABC',0)
        s.completed=[];s.bin_parts={k:[] for k in 'ABC'}
        pose=np.asarray(d['tcp']);b=r.bins['A']
        rotation,goal,q,start,_,_,route=s.plan_place(r.target,b,pose[:3,:3],pose[:3,3],pose[2,3],
                                                    payload_override=r.payload,from_current=True)
        center=goal+rotation@r.payload[0];extent=np.abs(rotation@r.payload[1])@r.payload[2]
        self.assertGreater(center[1]-d['old_release_center'][1],.018)
        self.assertGreaterEqual(center[1]-extent[1]-b.lower[1],.025-1e-8)
        self.assertGreaterEqual(b.upper[1]-center[1]-extent[1],.025-1e-8)
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,161):
                r.check_clearance(r.kin.matrix(a+(b-a)*fraction,1),1,payload=r.payload,extra_margin=.010)


if __name__=='__main__':unittest.main()
