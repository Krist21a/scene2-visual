"""Replay the actual grasp selector without publishing robot commands."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot
from scene2.perception import Part,Bin

ap=argparse.ArgumentParser();ap.add_argument('folder',type=Path);ap.add_argument('--urdf',type=Path,required=True)
a=ap.parse_args();rows=[json.loads(l) for l in (a.folder/'events.jsonl').read_text().splitlines()]
index=max(i for i,e in enumerate(rows) if e['event'] in ('select','target_refined'))
selected=rows[index];part=Part(**selected['part'])
observation=next(e for e in reversed(rows[:index]) if e['event']=='observation')
command=next(e for e in reversed(rows[:index]) if e['event']=='command')
nearest=min((abs(float(np.load(p)['stamp'])-part.stamp),p) for p in a.folder.glob('*.npz'))[1]
r=Robot.__new__(Robot);r.kin=UrdfArmKinematics(description=a.urdf.read_text())
r.command=np.asarray(command['command']);r.measured=lambda:np.asarray(command['measured'])
r.table=float(np.load(nearest)['table_z_base_m']);r.bins={k:Bin(**v) for k,v in observation['bins'].items()}
r.parts=[Part(**p) for p in observation['parts']];r.target=part;r.side=selected.get('side',1 if part.center[1]<0 else 0);r.held=False;r.payload=None;r.stage='replay'
r.parts=[part if p.kind==part.kind and np.linalg.norm(np.asarray(p.center)-part.center)<.025 else p for p in r.parts]
if not any(p is part for p in r.parts):r.parts.append(part)
def log(event,**values):
    if event in ('grasp_rejected','grasp_plan'):print(json.dumps(dict(event=event,**values),default=lambda v:np.asarray(v).tolist()),flush=True)
r.log=log;r.joints=lambda *args,**kwargs:None
hover=np.asarray(part.center).copy();hover[2]=r.table+.12
started=time.monotonic();r.orient_for_grasp(r.side,part,hover)
print(json.dumps(dict(elapsed_s=time.monotonic()-started,limitation='Grasp selector only; no motion or release-route validation.')))
