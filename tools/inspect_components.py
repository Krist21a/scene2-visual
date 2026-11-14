import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,cv2
from scene2.perception import *
s=dict(np.load(sys.argv[1]));points,valid=point_image(s);table=float(s['table_z_base_m']);pm,bm=masks(s['rgb']);parts,bins=Detector().detect(s,True)
print('detected',serialise(parts,bins))
for kind in (sys.argv[2],):
 surface=valid&(points[:,:,2]>table+.004)&(points[:,:,2]<table+.20)
 mask=cv2.morphologyEx((pm[kind]&surface).astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
 for region in components(mask,15):
  region&=valid&surface;p=points[region]
  if len(p)<15:continue
  center=np.median(p,axis=0)
  if not any(b.contains(center[:2],-.03) for b in bins.values()):continue
  _,vec=np.linalg.eigh(np.cov(p[:,:2].T));axis=vec[:,-1];basis=np.column_stack((axis,[-axis[1],axis[0]]));lo,hi=np.percentile(p[:,:2]@basis,[2,98],axis=0)
  print(json.dumps(dict(kind=kind,center=center.tolist(),extent=(hi-lo).tolist(),height=top_height(p,table),count=len(p))))
