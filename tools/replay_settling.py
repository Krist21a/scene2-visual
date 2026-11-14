"""Replay cavity continuity and settling feasibility from recorded RGB-D."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from scipy.spatial.transform import Rotation
from scene2.perception import Bin,masks
from scene2.bin_geometry import cavity_bounds,propagate_cavity
from scene2.settling import release_center,settling_fits
p=Path(sys.argv[1]);rows=[json.loads(l) for l in (p/'events.jsonl').read_text().splitlines()]
files=[(float(np.load(f)['stamp']),f) for f in p.glob('*.npz') if 'wrist' not in f.name]
reference=None
for event in rows:
    if event['event']!='observation' or 'C' not in event['bins']:continue
    bins={k:Bin(**v) for k,v in event['bins'].items()}
    distance,f=min((abs(stamp-bins['C'].stamp),f) for stamp,f in files)
    if distance>.02:raise ValueError('Missing matching head frame')
    snapshot=dict(np.load(f));c=cavity_bounds(snapshot,masks(snapshot['rgb'])[1]['C'],bins['C'])
    fresh=c.cavity is not None
    if fresh:reference=(c,bins)
    elif reference:c=propagate_cavity(*reference,bins) or c
    print(json.dumps(dict(elapsed_s=event['elapsed_s'],label=event.get('label'),fresh=fresh,supported=c.cavity is not None)),flush=True)
payload=next(e for e in reversed(rows) if e['event']=='payload_measured');half=np.asarray(payload['half']);feasible=[]
for pitch in (.65,.78):
    for angle in range(0,360,2):
        rot=Rotation.from_euler('z',np.deg2rad(angle)).as_matrix()@Rotation.from_euler('y',pitch).as_matrix()
        axis=rot[:,0];extent=np.abs(rot)@half
        try:xy=release_center(c,c,axis,half,extent,.01)
        except ValueError:continue
        if settling_fits(c,np.r_[xy,0],axis,half):feasible.append([pitch,angle])
print(json.dumps(dict(feasible=feasible,limitation='Geometry only; IK, existing objects, motion and final acceptance still require validation.')),flush=True)
raise SystemExit(0 if feasible else 1)
