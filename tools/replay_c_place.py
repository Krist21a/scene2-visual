"""Read-only release planner replay from recorded camera and joint evidence."""
import argparse
import json
import sys
import time
from dataclasses import replace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot
from scene2.perception import Part, Bin, masks, Detector
from scene2.bin_geometry import cavity_bounds,propagate_cavity
from scene2.task import Sorter,fits_bin
from scene2.settling import settled_corners, settling_fits,support_bias

ap=argparse.ArgumentParser()
ap.add_argument('folder',type=Path)
ap.add_argument('--urdf',type=Path,required=True)
ap.add_argument('--target',type=int,default=-1)
args=ap.parse_args()
rows=[json.loads(line) for line in (args.folder/'events.jsonl').read_text().splitlines()]
targets=[i for i,e in enumerate(rows) if e['event']=='target_refined' and e['part']['kind']=='C']
index=targets[args.target]
transport=next(i for i in range(index,len(rows)) if rows[i]['event']=='transport_start')
event=rows[transport]
part=Part(**rows[index]['part'])
observation=next(e for e in reversed(rows[:index]) if e['event']=='observation')
command=next(e for e in reversed(rows[:transport]) if e['event']=='command')
snapshots=[(abs(float(np.load(p)['stamp'])-part.stamp),p) for p in args.folder.glob('*-pregrasp.npz')]
snapshot=dict(np.load(min(snapshots)[1]))
r=Robot.__new__(Robot)
r.kin=UrdfArmKinematics(description=args.urdf.read_text())
r.command=np.asarray(command['command'])
measured=np.asarray(command['measured']);measured[:7]=event['q']
r.measured=lambda:measured.copy()
r.table=float(snapshot['table_z_base_m'])
r.bins={k:Bin(**v) for k,v in observation['bins'].items()}
r.parts=[Part(**p) for p in observation['parts']]
r.parts=[part if p.kind=='C' and np.linalg.norm(np.asarray(p.center)-part.center)<.025 else p for p in r.parts]
r.target=part;r.side=0;r.held=True;r.stage='replay'
r.payload=tuple(np.asarray(v) for v in event['payload']);r.payload_shaft_sign=1.
def log(name,**values):
    if name in ('place_plan','place_rejected'):
        print(json.dumps(dict(event=name,**values),default=lambda v:np.asarray(v).tolist()),flush=True)
r.log=log
s=Sorter.__new__(Sorter);s.robot=r;s.log=log;s.initial_counts=dict.fromkeys('ABC',0);s.completed=[]
s.initial_counts=next((e['counts'] for e in rows[:index] if e['event']=='initial_occupancy'),s.initial_counts)
s.bin_parts={k:[] for k in 'ABC'}
for e in rows[:index]:
    if e['event']=='completed':s.completed=e['ledger']
prior_files=[(float(np.load(f)['stamp']),f) for f in args.folder.glob('*.npz')
             if (f.name.endswith('-placement_check.npz') or f.name.endswith('-initial_occupancy.npz'))
             if float(np.load(f)['stamp'])<part.stamp]
if prior_files:
    prior=dict(np.load(max(prior_files)[1]));parts,prior_bins=Detector().detect(prior,True)
    before=np.array([prior_bins[k].center for k in ('A','B')]);after=np.array([r.bins[k].center for k in ('A','B')])
    u,_,vt=np.linalg.svd((before-before.mean(0)).T@(after-after.mean(0)));registration=vt.T@u.T
    if np.linalg.det(registration)<0:vt[-1]*=-1;registration=vt.T@u.T
    translation=after.mean(0)-registration@before.mean(0)
    if abs(np.arctan2(registration[1,0],registration[0,0]))>.05 or np.linalg.norm(translation)>.05:
        raise ValueError('Recorded occupied geometry registration inconsistent')
    dz=r.table-float(prior['table_z_base_m'])
    def transform(point):return np.r_[registration@np.asarray(point)[:2]+translation,point[2]+dz].tolist()
    for p in parts:
        if fits_bin(p,prior_bins[p.kind]):
            s.bin_parts[p.kind].append(replace(p,center=transform(p.center),full_center=transform(p.full_center),
                axis=(registration@p.axis).tolist(),top=p.top+dz,
                footprint=(np.asarray(p.footprint)@registration.T+translation).tolist()))
s.release_bins=dict(r.bins)
s.release_bins['C']=cavity_bounds(snapshot,masks(snapshot['rgb'])[1]['C'],r.bins['C'])
if s.release_bins['C'].cavity is None and prior_files:
    s.release_bins['C']=propagate_cavity(prior_bins['C'],prior_bins,r.bins) or s.release_bins['C']
recorded=next((e for e in reversed(rows[:index]) if e['event']=='release_cavity'),None)
propagated=next((e for e in reversed(rows[:index]) if e['event']=='release_cavity_propagated'),None)
if recorded is not None:
    s.release_bins['C']=Bin(**recorded['bin'])
elif propagated is not None:
    distance,reference_file=min((abs(float(np.load(f)['stamp'])-propagated['reference_stamp']),f)
                                for f in args.folder.glob('*.npz'))
    if distance>1e-5:raise ValueError('Missing exact cavity reference image')
    reference=dict(np.load(reference_file));_,reference_bins=Detector().detect(reference)
    region=cavity_bounds(reference,masks(reference['rgb'])[1]['C'],reference_bins['C'])
    s.release_bins['C']=propagate_cavity(region,reference_bins,r.bins)
    if s.release_bins['C'] is None:raise ValueError('Recorded cavity propagation cannot be reproduced')
started=time.monotonic()
tcp=np.asarray(event['tcp'])
result=s.plan_place(part,r.bins['C'],tcp[:3,:3],tcp[:3,3],tcp[:3,3],payload_override=r.payload,from_current=True)
rotation,goal=result[:2]
center=goal+rotation@r.payload[0];axis=(rotation@r.payload[1])[:,0]
bias=None
print(json.dumps(dict(elapsed_s=time.monotonic()-started,settling_fits=settling_fits(s.release_bins['C'],center,axis,r.payload[2],handle_length=part.extent[0],bias=bias),
    center=center.tolist(),settled_corners=settled_corners(center,axis,r.payload[2],part.extent[0],bias).tolist(),support_bias=bias,
    occupied={k:len(v) for k,v in s.bin_parts.items()},limitation='Recorded geometry and registered occupied objects only; live validation required.')),flush=True)
