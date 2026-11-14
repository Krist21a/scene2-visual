"""Read-only pose feasibility using a recorded held pose and public URDF."""
import argparse
import collections
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from scipy.spatial.transform import Rotation
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot, Stop
from scene2.perception import Part, Bin, Detector
from scene2.release import free_drop_clear
from scene2.settling import release_center
from scene2.task import RELEASE_JOINT_MARGIN

ap = argparse.ArgumentParser()
ap.add_argument('folder', type=Path)
ap.add_argument('--urdf', type=Path, required=True)
a = ap.parse_args()
rows = [json.loads(line) for line in (a.folder / 'events.jsonl').read_text().splitlines()]
e = next(e for e in rows if e['event'] == 'transport_start')
obs = next(e for e in rows if e['event'] == 'observation' and e['label'] == 'pregrasp')
part = Part(**next(e for e in rows if e['event'] == 'target_refined')['part'])
r = Robot.__new__(Robot)
r.kin = UrdfArmKinematics(description=a.urdf.read_text())
r.command = np.zeros(16); r.command[7] = 1
r.side = 0; r.target = part; r.held = True; r.stage = 'survey'; r.payload_shaft_sign = 1.
r.payload = tuple(np.array(v) for v in e['payload'])
r.bins = {k: Bin(**v) for k, v in obs['bins'].items()}
r.parts = [Part(**p) for p in obs['parts']]; r.settled_parts = []
r.log = lambda *args, **kwargs: None
snapshot = dict(np.load(next(a.folder.glob('*-pregrasp.npz'))))
r.table = float(snapshot['table_z_base_m'])
_, bins = Detector().detect(snapshot, True)
cavity = bins['C']; bin = r.bins['C']
offset, local, half = r.payload
roll0 = np.arctan2(local.T[1, 0], local.T[2, 0])
failures = collections.Counter(); found = []
for pitch in (.26, .40, .52, .65):
    for degrees in range(40, 361, 10):
        for delta in (-np.pi/2, np.pi/2, -np.pi/3, np.pi/3):
            rot = (Rotation.from_euler('z', np.deg2rad(degrees)).as_matrix() @
                   Rotation.from_euler('y', pitch).as_matrix() @
                   Rotation.from_euler('x', roll0 + delta).as_matrix() @ local.T)
            extent = np.abs(rot @ local) @ half
            center = np.r_[bin.center, bin.rim + extent[2] + .045]
            try:
                center[:2] = release_center(cavity, cavity, (rot@local)[:,0], half, extent, .010,
                                           part.extent[0], rot@local)
                pose = np.eye(4); pose[:3,:3] = rot; pose[:3,3] = center-rot@offset
                if not free_drop_clear(pose, r.payload, part.extent[0], r.table):
                    failures['finger_support'] += 1; continue
                r.check_clearance(pose, 0, payload=r.payload, extra_margin=.010, gripper_open=True)
                q = r.solve_pose(0, pose[:3,3], rot, np.array(e['q']), joint_margin=RELEASE_JOINT_MARGIN)
                for fraction in np.linspace(0,1,31):
                    r.check_clearance(r.kin.matrix(np.array(e['q'])*(1-fraction)+q*fraction,0),0,
                                      payload=r.payload,extra_margin=.010)
                found.append(dict(pitch=pitch,yaw=degrees,roll_delta=delta,q=q.tolist(),goal=pose[:3,3].tolist()))
                print(json.dumps(found[-1]),flush=True)
                if len(found)>=8:break
            except (ValueError,Stop) as ex: failures[str(ex)]+=1
        if len(found)>=8:break
    if len(found)>=8:break
print(json.dumps(dict(found=len(found),failures=dict(failures))),flush=True)
