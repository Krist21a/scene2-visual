"""Guarded empty-hand development recovery using live and registered visual geometry."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy,yaml
from dataclasses import replace
from scene1.rgbd import RgbdScene
from scene2.control import Robot,Stop
from scene2.task import Sorter
from scene2.perception import Part
rospy.init_node('scene2_park_empty',anonymous=True)
def log(event,**values):
 if event!='command':print(json.dumps(dict(event=event,**values),default=lambda v:v.tolist()),flush=True)
scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
r=sorter=None
try:
 r=Robot(rospy,log);sorter=Sorter(r,scene,log,lambda *a:None)
 s,parts,bins=sorter.observe('recovery')
 rows=[json.loads(l) for l in Path(sys.argv[1]).read_text().splitlines()]
 old=next(x for x in rows if x['event']=='observation' and x['label']=='select')
 keys=sorted(bins)
 a=np.array([old['bins'][k]['center'] for k in keys]);b=np.array([bins[k].center for k in keys])
 u,_,vt=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)));rot=vt.T@u.T
 if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
 translation=b.mean(0)-rot@a.mean(0)
 if np.max(np.linalg.norm(a@rot.T+translation-b,axis=1))>.015:raise Stop('Recovery table registration inconsistent')
 dz=r.table-(old['bins']['A']['floor']-.01)
 def point(v):return np.r_[rot@np.array(v)[:2]+translation,v[2]+dz].tolist()
 for entry in old['parts']:
  if entry['kind']!='A':continue
  p=Part(**entry)
  p=replace(p,center=point(p.center),full_center=point(p.full_center),axis=(rot@p.axis).tolist(),top=p.top+dz,
            footprint=(np.array(p.footprint)@rot.T+translation).tolist(),occupied_xy=(np.array(p.occupied_xy)@rot.T+translation).tolist())
  if not any(q.kind=='A' and np.linalg.norm(np.array(q.center)-p.center)<.03 for q in r.parts):r.parts.append(p)
 r.side=1
 r.joint_path(np.deg2rad([30,10,10,-120,-60,0,0,30,-10,-10,-120,60,0,0]),'park_empty')
 print('PARKED',flush=True)
finally:
 if sorter:sorter.close()
 scene.close()
 if r:r.close()
